/**
 * \file vts/storage/storage.cpp
 * \author Vaclav Blazek <vaclav.blazek@citationtech.net>
 *
 * Tile set storage change operations.
 * Not present in vts-core library.
 */

#include <memory>
#include <string>
#include <exception>
#include <algorithm>
#include <iterator>

#include <boost/optional.hpp>
#include <boost/noncopyable.hpp>
#include <boost/filesystem.hpp>
#include <boost/format.hpp>
#include <boost/range/adaptor/reversed.hpp>

#include "utility/streams.hpp"
#include "utility/openmp.hpp"
#include "utility/path.hpp"
#include "utility/duration.hpp"
#include "utility/time.hpp"

#ifdef UTILITY_HAS_PROC
#  include "utility/procstat.hpp"
#endif

#include "../../storage/error.hpp"
#include "../../storage/tidguard.hpp"
#include "../storage.hpp"
#include "../../vts.hpp"
#include "./detail.hpp"
#include "../tileset/detail.hpp"
#include "../encoder.hpp"
#include "../io.hpp"

#include "./config.hpp"
#include "./paths.hpp"

namespace fs = boost::filesystem;

namespace vadstena { namespace vts {

namespace {

#ifdef UTILITY_HAS_PROC

void reportMemoryUsage(const std::string &what)
{
    const double kb2gb(1 << 20);
    auto stat(utility::getProcStat());
    LOG(info3)
        << std::fixed
        << "Memory occupied " << what << ": " << (stat.occupies() / kb2gb)
        << " GiB (of which " << (stat.swap / kb2gb) << " GiB is swapped).";
}

#else

void reportMemoryUsage(const std::string&) {}

#endif

} // namespace

void Storage::add(const boost::filesystem::path &tilesetPath
                  , const Location &where, const TilesetId &tilesetId
                  , const AddOptions &addOptions)
{
    auto ts(openTileSet(tilesetPath));
    detail().add(ts, where
                 , (tilesetId.empty() ? ts.getProperties().id : tilesetId)
                 , addOptions);
}

void Storage::readd(const TilesetId &tilesetId
                    , const AddOptions &addOptions)
{
    detail().readd(tilesetId, addOptions);
}

void Storage::remove(const TilesetIdList &tilesetIds)
{
    detail().remove(tilesetIds);
}

namespace {

void rmrf(const fs::path &path)
{
    boost::system::error_code ec;
    remove_all(path, ec);
}

class Tx {
public:
    Tx(const fs::path &root) : root_(root) {}
    ~Tx();

    void add(const fs::path &work, const fs::path &dst);

    const fs::path root() const { return root_; }

    fs::path tilesetPath(const std::string &tilesetId, bool tmp = false)
        const
    {
        return createPath(storage_paths::tilesetPath(root_, tilesetId, tmp));
    }

    fs::path gluePath(const Glue &glue, bool tmp = false)
        const
    {
        return createPath(storage_paths::gluePath(root_, glue, tmp));
    }

    TileSet open(const TilesetId &tilesetId) const;

    TileSet open(const Glue &glue) const;

    fs::path addGlue(const Glue &glue);

    fs::path addTileset(const std::string &tilesetId);

    void remove(const fs::path &path);

    void commit();

private:
    void rollback();

    fs::path createPath(const fs::path &path) const;

    const fs::path root_;
    typedef std::map<fs::path, fs::path> Mapping;
    Mapping mapping_;
};

Tx::~Tx() {
    if (std::uncaught_exception()) {
        // we cannot throw!
        rollback();
    }
}

void Tx::add(const fs::path &work, const fs::path &dst)
{
    mapping_.insert(Mapping::value_type(work, dst));
}

void Tx::rollback()
{
    for (const auto &item : mapping_) {
        try { rmrf(item.first); } catch (...) {}
    }
    mapping_.clear();
}

void Tx::commit()
{
    for (const auto &item : mapping_) {
        if (item.second.empty()) {
            LOG(info2) << "commit(rm(" << item.first << "))";
            // no destination -> just remove
            rmrf(item.first);
        } else {
            LOG(info2)
                << "commit(rm(" << item.second << "), "
                << "mv(" << item.first << ", " << item.second << "))";
            // remove old stuff
            rmrf(item.second);
            // move new stuff there
            rename(item.first, item.second);
        }
    }
    mapping_.clear();
}

TileSet Tx::open(const TilesetId &tilesetId) const
{
    return openTileSet(tilesetPath(tilesetId));
}

TileSet Tx::open(const Glue &glue) const
{
    return openTileSet(gluePath(glue));
}

fs::path Tx::addGlue(const Glue &glue)
{
    auto tmp(gluePath(glue, true));
    add(tmp, gluePath(glue));
    return tmp;
}

fs::path Tx::addTileset(const std::string &tilesetId)
{
    auto tmp(tilesetPath(tilesetId, true));
    add(tmp, tilesetPath(tilesetId));
    return tmp;
}

fs::path Tx::createPath(const fs::path &path) const
{
    fs::create_directories(path.parent_path());
    return path;
}

void Tx::remove(const fs::path &path)
{
    mapping_[path] = fs::path();
}

} // namespace


namespace {

typedef std::vector<TileSet> TileSets;
typedef std::vector<TileIndex> TileIndices;

std::tuple<TileSets, std::size_t>
openTilesets(Tx &tx, const StoredTileset::list &infos, const TileSet &tileset
             , const vts::TilesetId &addedId)
{
    std::tuple<TileSets, std::size_t> res;
    TileSets &tilesets(std::get<0>(res));
    std::size_t index(0);
    for (const auto &info : infos) {
        if (info.tilesetId == addedId) {
            tilesets.push_back(tileset);
            std::get<1>(res) = index;
            LOG(info2) << "Reused already open <" << addedId << ">.";
        } else {
            tilesets.push_back(tx.open(info.tilesetId));
            LOG(info2) << "Opened tileset <" << info.tilesetId << ">.";
        }

        ++index;
    }
    return res;
}

LodRange range(const TileSets &tilesets)
{
    auto lr(LodRange::emptyRange());
    for (const auto &ts : tilesets) {
        lr = unite(lr, ts.lodRange());
    }
    return lr;
}

struct Ts {
    /** Own index in the list of tilesets.
     */
    std::size_t index;

    /** Marks added tileset.
     */
    bool added;

    /** This tileset
     */
    TileSet set;

    /** Stored ID.
     */
    StoredTileset storedId;

    /** Set of tilesets that overlap with this one.
     */
    std::set<std::size_t> incidentSets;

    Ts(int index, const TileSet &tileset, const LodRange &lodRange
       , const Storage::Properties &properties, bool added)
        : index(index), added(added), set(tileset)
        , storedId(properties.tilesets[index])
        , lodRange(lodRange)
    {}

    bool notoverlaps(const Ts &other) const {
        return sphereOfInfluence().notoverlaps
            (other.sphereOfInfluence(), TileIndex::Flag::any);
    }

    std::string id() const { return storedId.tilesetId; }

    std::string base() const { return storedId.baseId; }

    const TileIndex& sphereOfInfluence() const {
        if (!sphereOfInfluence_) {
            sphereOfInfluence_ =
                (set.sphereOfInfluence(lodRange, TileIndex::Flag::mesh));
        }
        return *sphereOfInfluence_;
    }

    typedef std::vector<Ts> list;

    typedef std::vector<Ts*> ptrlist;
    typedef std::vector<const Ts*> const_ptrlist;

private:
    /** Tileset's sphere of influence.
     */
    mutable boost::optional<TileIndex> sphereOfInfluence_;

    LodRange lodRange;
};

class BitSet {
public:
    BitSet(std::size_t size = 0) : bs_(size, 0) {}

    bool increment();

    typedef std::vector<bool> repr_type;

    const repr_type& repr() const { return bs_; }

    typedef repr_type::iterator iterator;
    typedef repr_type::const_iterator const_iterator;

    iterator begin() { return bs_.begin(); }
    iterator end() { return bs_.end(); }
    const_iterator begin() const { return bs_.begin(); }
    const_iterator end() const { return bs_.end(); }
    const_iterator cbegin() { return bs_.begin(); }
    const_iterator cend() { return bs_.end(); }
    repr_type::reference operator[](std::size_t index) { return bs_[index]; }
    repr_type::const_reference operator[](std::size_t index) const {
        return bs_[index];
    }

private:
    std::vector<bool> bs_;
};

template<typename CharT, typename Traits>
inline std::basic_ostream<CharT, Traits>&
operator<<(std::basic_ostream<CharT, Traits> &os, const BitSet &bs)
{
    for (auto bit : boost::adaptors::reverse(bs.repr())) {
        os << (bit ? '1' : '0');
    }
    return os;
}

bool BitSet::increment()
{
    for (std::size_t i(0), e(bs_.size()); i != e; ++i) {
        auto nval(!bs_[i]);
        bs_[i] = nval;
        if (nval) { return true; }
    }
    return false;
}

struct GlueDescriptor {
    TileSet::const_ptrlist combination;
    Glue glue;
    TilesetId glueSetId;

    GlueDescriptor(const TileSet::const_ptrlist &combination
                   , const Glue &glue, const TilesetId &glueSetId)
        : combination(combination), glue(glue), glueSetId(glueSetId)
    {}

    typedef std::vector<GlueDescriptor> list;
};

GlueDescriptor::list prepareGlues(Ts::list &tilesets, Ts &added)
{
    Ts::ptrlist incidentSets;
    {
        for (auto &ts : tilesets) {
            // ignore added tileset
            if (ts.added) {continue; }
            if (ts.notoverlaps(added)) {
                LOG(info1) << "Tileset <" << ts.id()
                           << "> does not overlap added tileset <"
                           << added.id() << ">.";
                continue;
            }

            // incidence between spheres of influence -> remember
            LOG(info1) << "Adding <" << ts.id() << "> to incident set.";
            incidentSets.push_back(&ts);
        }
    }

    // for each tileset in the input
    {
        for (auto iincidentSets(incidentSets.begin())
                 , eincidentSets(incidentSets.end());
             iincidentSets != eincidentSets; ++iincidentSets)
        {
            auto &first(**iincidentSets);

            // process all remaining tilesets, add self
            for (auto iincidentSets2(std::next(iincidentSets));
                 iincidentSets2 != eincidentSets; ++iincidentSets2)
            {
                auto &second(**iincidentSets2);

                // overlaps -> remember
                if (!first.notoverlaps(second)) {
                    first.incidentSets.insert(second.index);
                }
            }
        }
    }

    // result
    GlueDescriptor::list gd;

    for (const auto &tsp : incidentSets) {
        const auto &ts(*tsp);

        LOG(info2)
            << "Sets <" << added.id() << "> and <"
            << ts.id() << "> (" << added.index << ", " << ts.index
            << ") overlap with [" << utility::join(ts.incidentSets, ", ")
            << "].";

        // initialize tileset mapping
        constexpr int emptyPlaceholder(-1);
        constexpr int addedPlaceholder(-2);
        constexpr int thisPlaceholder(-3);

        std::vector<int> mapping(tilesets.size(), emptyPlaceholder);
        mapping[ts.index] = thisPlaceholder;
        mapping[added.index] = addedPlaceholder;
        {
            int index(0);
            for (auto its : ts.incidentSets) {
                mapping[its] = index;
                ++index;
            }
        }

        // build all combinations (ts, added and any combination of incident
        // sets)
        BitSet flags(ts.incidentSets.size());

        do {
            TileSet::const_ptrlist combination;
            Glue glue;

            auto buildCombination([&]() -> bool
            {
                std::set<TilesetId> seen;
                auto addTs([&](const Ts &ts) -> bool
                {
                    if (!seen.insert(ts.base()).second) {
                        // this base tileset has already been seen
                        return false;
                    }
                    combination.push_back(&ts.set);
                    glue.id.push_back(ts.id());
                    return true;
                });

                for (std::size_t index(0), e(mapping.size());
                     index != e; ++ index)
                {
                    auto value(mapping[index]);
                    switch (value) {
                    case emptyPlaceholder:
                        continue;

                    case addedPlaceholder:
                        if (!addTs(added)) {
                            return false;
                        }
                        continue;

                    case thisPlaceholder:
                        if (!addTs(*tsp)) {
                            return false;
                        }
                        continue;
                    }

                    if (flags[value] && !addTs(tilesets[index])) {
                        return false;
                    }
                }
                return true;
            });

            if (!buildCombination()) { continue; }

            // create glue
            auto glueSetId(boost::lexical_cast<std::string>
                           (utility::join(glue.id, "_")));
            glue.path = glueSetId;

            gd.emplace_back(combination, glue, glueSetId);
        } while (flags.increment());
    }

    return gd;
}

Storage::Properties
createGlues(Tx &tx, Storage::Properties properties
            , const std::tuple<TileSets, std::size_t> &tsets
            , const Storage::AddOptions &addOptions)
{
    if (properties.tilesets.size() <= 1) {
        LOG(info3) << "No need to create any glue.";
        return properties;
    }

    // create tileset list
    Ts::list tilesets;
    {
        // accumulate lod range for all tilesets
        auto lr(range(std::get<0>(tsets)));
        LOG(info1) << "Glue lod range: " << lr;

        std::size_t index(0);
        for (auto &set : std::get<0>(tsets)) {
            tilesets.emplace_back(index, set, lr, properties
                                  , (index == std::get<1>(tsets)));
            ++index;
        }
    }

    // prepare glues
    auto gds(prepareGlues(tilesets, tilesets[std::get<1>(tsets)]));

    {
        int glueNumber(1);
        LOG(info3) << "Will try to generate " << gds.size() << "glues:";
        for (const auto &gd : gds) {
            LOG(info3)
                << "    #" << glueNumber
                << '/' << gds.size() << " <" << gd.glueSetId << ">.";
            ++glueNumber;
        }
    }

    // simulation -> stop here
    if (addOptions.dryRun) { return properties; }

    int glueNumber(1);
    for (const auto &gd : gds) {
        LOG(info3)
            << "Trying to generate glue #" << glueNumber
            << '/' << gds.size() << " <" << gd.glueSetId << ">.";
        vadstena::storage::TIDGuard tg
            (str(boost::format("%d:%s") % glueNumber % gd.glueSetId)
             , true);

        ++glueNumber;

        TileSetProperties gprop;
        gprop.id = gd.glueSetId;
        gprop.referenceFrame
            = gd.combination.front()->getProperties().referenceFrame;

        reportMemoryUsage("before glue creation");

        // create glue
        auto tmpPath(tx.addGlue(gd.glue));
        auto gts(createTileSet(tmpPath, gprop, CreateMode::overwrite));

        // create glue
        utility::DurationMeter timer;
        gts.createGlue(gd.combination, addOptions.textureQuality);
        auto duration(timer.duration());

        reportMemoryUsage("after merge");

        // empty cached input data (no need for them now)
        for (const auto *ts : gd.combination) { ts->emptyCache(); }

        reportMemoryUsage("after emptying input caches");

        if (gts.empty()) {
            // unusable
            LOG(info3)
                << "Glue <" << gd.glueSetId  << "> contains no tile; "
                << "glue is forgotten. Duration: "
                << utility::formatDuration(duration) << ".";
            tx.remove(tmpPath);
        } else {
            // usable
            LOG(info3)
                << "Glue <" << gd.glueSetId  << "> created, duration: "
                << utility::formatDuration(duration) << ".";

            // flush
            gts.flush();

            reportMemoryUsage("after glue flush");

            // and remember
            properties.glues[gd.glue.id] = gd.glue;
        }

        reportMemoryUsage("after glue creation");
    }

    return properties;
}

} // namespace

std::tuple<Storage::Properties, StoredTileset>
Storage::Detail::addTileset(const Properties &properties
                            , const TilesetId &tilesetId, bool bumpVersion
                            , const Location &where) const
{
    if (tilesetId.find('@') != std::string::npos) {
        LOGTHROW(err1, vadstena::storage::TileSetAlreadyExists)
            << "Invalid character in tileset ID <" << tilesetId << ">.";
    }

    StoredTileset tileset;
    {
        tileset.baseId = tilesetId;
        auto lastVersion(properties.lastVersion(tilesetId));
        if (lastVersion >= 0) {
            if (!bumpVersion) {
                LOGTHROW(err1, vadstena::storage::TileSetAlreadyExists)
                    << "Tileset <" << tilesetId
                    << "> already present in storage "
                    << root << ". Use version bumping to introduce "
                    "new version.";
            }

            tileset.version = lastVersion + 1;
            tileset.baseId = tilesetId;
            tileset.tilesetId = str(boost::format("%s@%d")
                                    % tilesetId % tileset.version);
            LOG(info2)
                << "Bumping version " << tileset.version
                << " of tileset <" << tileset.baseId << "> under name ID <"
                << tileset.tilesetId << ">";
        } else {
            tileset.baseId = tileset.tilesetId = tilesetId;
            tileset.version = 0;
        }
    }

    if (properties.hasTileset(tileset.tilesetId)) {
        LOGTHROW(err1, vadstena::storage::TileSetAlreadyExists)
            << "Tileset <" << tileset.tilesetId
            << "> already present in storage "
            << root << ".";
    }

    std::tuple<Properties, StoredTileset> res;
    auto &p(std::get<0>(res) = properties);
    std::get<1>(res) = tileset;

    auto &tilesets(p.tilesets);

    if (where.where.empty()) {
        // void reference
        if (where.direction == Location::Direction::below) {
            // below void -> to the top of the stack
            tilesets.push_back(tileset);
        } else {
            // above void -> to the bottom of the stack
            tilesets.insert(tilesets.begin(), tileset);
        }
        return res;
    }

    // some reference
    auto ftilesets(p.findTilesetIt(where.where));
    if (ftilesets == tilesets.end()) {
        LOGTHROW(err1, vadstena::storage::NoSuchTileSet)
            << "Tileset <" << where.where << "> (used as a reference) "
            "not found in storage " << root << ".";
    }

    if (where.direction == Location::Direction::below) {
        // below given reference -> just insert here
        tilesets.insert(ftilesets, tileset);
    } else {
        // above reference -> advance and insert
        tilesets.insert(std::next(ftilesets), tileset);
    }

    return res;
}

std::tuple<Storage::Properties, Glue::map>
Storage::Detail::removeTilesets(const Properties &properties
                                , const TilesetIdList &tilesetIds)
    const
{
    std::tuple<Properties, Glue::map> res(properties, {});
    auto &p(std::get<0>(res));
    auto &tilesets(p.tilesets);

    for (const auto &tilesetId : tilesetIds) {
        auto ftilesets(p.findTilesetIt(tilesetId));
        if (ftilesets == tilesets.end()) {
            LOG(warn2) << "Tileset <" << tilesetId << "> "
                "not found in storage " << root << ".";
        } else {
            // remove from tilesets
            tilesets.erase(ftilesets);
        }
    }

    // drop all glues that reference requested tilesets
    auto &glues(p.glues);
    auto &resGlues(std::get<1>(res));
    for (const auto &tilesetId : tilesetIds) {
        for (auto iglues(glues.begin()); iglues != glues.end(); ) {
            if (iglues->second.references(tilesetId)) {
                resGlues.insert(*iglues);
                iglues = glues.erase(iglues);
            } else {
                ++iglues;
            }
        }
    }

    return res;
}

void Storage::Detail::add(const TileSet &tileset, const Location &where
                          , const TilesetId &tilesetId
                          , const AddOptions &addOptions)
{
    std::string simulation(addOptions.dryRun ? "(simulation) " : "");
    vadstena::storage::TIDGuard tg
        (str(boost::format("%sadd(%s)") % simulation % tilesetId));

    // check compatibility
    if (tileset.getProperties().referenceFrame != properties.referenceFrame) {
        LOGTHROW(err1, vadstena::storage::IncompatibleTileSet)
            << "Tileset <" << tilesetId << "> "
            "uses different reference frame ("
            << tileset.getProperties().referenceFrame
            << ") from the one  this storage supports ("
            << properties.referenceFrame << ").";
    }

    // prepare new tileset list
    Properties nProperties;
    StoredTileset tilesetInfo;
    std::tie(nProperties, tilesetInfo)
        = addTileset(properties, tilesetId, addOptions.bumpVersion, where);


    LOG(info3)
        << "Adding tileset <" << tileset.id() << "> (from "
        << tileset.root() << ").";

    {
        Tx tx(root);

        auto dst([&]() -> TileSet
        {
            if (addOptions.dryRun) { return tileset; }

            // create tileset at work path (overwrite any existing stuff here)
            // NB: we have to clone original tileset's content as-is!
            return cloneTileSet(tx.addTileset(tilesetInfo.tilesetId), tileset
                                , CloneOptions()
                                .mode(CreateMode::overwrite)
                                .sameType(true)
                                .tilesetId(tilesetInfo.tilesetId)
                                .lodRange(addOptions.filter.lodRange()));
        }());

        auto tilesets(openTilesets(tx, nProperties.tilesets
                                   , dst, tilesetInfo.tilesetId));
        nProperties = createGlues(tx, nProperties, tilesets
                                  , addOptions);

        // dry run -> do nothing
        if (addOptions.dryRun) { return; }

        // commit changes
        tx.commit();
    }

    // commit properties
    properties = nProperties;
    saveConfig();
}

void Storage::Detail::readd(const TilesetId &tilesetId
                            , const AddOptions &addOptions)
{
    std::string simulation(addOptions.dryRun ? "(simulation) " : "");
    vadstena::storage::TIDGuard tg
        (str(boost::format("%sreadd(%s)") % simulation % tilesetId));

    LOG(info3) << "Readding tileset <" << tilesetId << ">.";

    auto nProperties(properties);
    {
        Tx tx(root);

        auto dst(openTileSet(storage_paths::tilesetPath(root, tilesetId)));

        // create glues only if tileset participates in any glue
        auto tilesets(openTilesets(tx, properties.tilesets, dst, dst.id()));
        nProperties = createGlues(tx, nProperties, tilesets, addOptions);

        // dry run -> do nothing
        if (addOptions.dryRun) { return; }

        tx.commit();
    }

    // commit properties
    properties = nProperties;
    saveConfig();
}

void Storage::Detail::remove(const TilesetIdList &tilesetIds)
{
    // dbglog::thread_id(str(boost::format("%s/%s->void")
    //                       % root.filename().string()
    //                       % tilesetId));

    Properties nProperties;
    Glue::map glues;
    std::tie(nProperties, glues) = removeTilesets(properties, tilesetIds);

    LOG(info3)
        << "Removing tilesets <" << utility::join(tilesetIds, ", ") << ">.";

    properties = nProperties;
    saveConfig();

    // physical removal
    for (const auto &tilesetId : tilesetIds) {
        auto path(storage_paths::tilesetPath(root, tilesetId));
        LOG(info3) << "Removing " << path << ".";
        rmrf(path);
    }

    for (const auto &item : glues) {
        const auto &glue(item.second);
        auto path(storage_paths::gluePath(root, glue));
        LOG(info3) << "Removing glue " << glue.path << ".";
        rmrf(path);
    }
}

} } // namespace vadstena::vts

