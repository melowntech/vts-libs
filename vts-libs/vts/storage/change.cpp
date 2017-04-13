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
#include <functional>

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
#include "utility/expect.hpp"

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
#include "./gluerules.hpp"
#include "./mergeconf.hpp"
#include "./locking.hpp"

namespace fs = boost::filesystem;

namespace vtslibs { namespace vts {

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

inline std::string glueId2path(const Glue::Id &id)
{
    return boost::lexical_cast<std::string>(utility::join(id, "_"));
}

Storage::AddOptions updateAddOptions(Storage::AddOptions addOptions
                                     , const MergeConf &mergeConf)
{
    addOptions.openOptions.updateCNames(mergeConf.cnames);
    addOptions.openOptions.scarceMemory(true);
    return addOptions;
}

inline std::string lockName(const TilesetId &tilesetId) { return tilesetId; }
inline std::string lockName(const Glue &glue) { return glue.path; }
inline std::string lockName(const VirtualSurface &virtualSurface) {
    return "#" + virtualSurface.path;
}

} // namespace

void Storage::add(const boost::filesystem::path &tilesetPath
                  , const Location &where, const TilesetId &tilesetId
                  , const AddOptions &addOptions)
{
    const auto ao
        (updateAddOptions
         (addOptions
          , loadMergeConf(detail().root / storage_paths::mergeConfPath()
                          , true)));

    auto ts(openTileSet(tilesetPath, ao.openOptions));
    detail().add(ts, where
                 , (tilesetId.empty() ? ts.getProperties().id : tilesetId)
                 , ao);
}

void Storage::generateGlues(const TilesetId &tilesetId
                            , const AddOptions &addOptions)
{
    const auto ao
        (updateAddOptions
         (addOptions
          , loadMergeConf(detail().root / storage_paths::mergeConfPath()
                          , true)));

    detail().generateGlues(tilesetId, ao);
}

void Storage::generateGlue(const Glue::Id &glueId
                           , const AddOptions &addOptions)
{
    const auto ao
        (updateAddOptions
         (addOptions
          , loadMergeConf(detail().root / storage_paths::mergeConfPath()
                          , true)));

    detail().generateGlue(detail().properties.normalize(glueId)
                          , ao);
}

void Storage::remove(const TilesetIdList &tilesetIds
                     , const StorageLocker::pointer &locker)
{
    detail().remove(tilesetIds, locker);
}

void Storage::createVirtualSurface(const TilesetIdSet &tilesets
                                   , CreateMode mode
                                   , const StorageLocker::pointer &locker)
{
    detail().createVirtualSurface(tilesets, mode, locker);
}

void Storage::removeVirtualSurface(const TilesetIdSet &tilesets
                                   , const StorageLocker::pointer &locker)
{
    detail().removeVirtualSurface(tilesets, locker);
}

namespace {

void rmrf(const fs::path &path)
{
    boost::system::error_code ec;
    remove_all(path, ec);
}

class Tx : boost::noncopyable {
public:
    Tx(const fs::path &root, const boost::optional<fs::path> &tmpRoot
       , const OpenOptions &openOptions = OpenOptions())
        : root_(root), tmpRoot_(tmpRoot), openOptions_(openOptions)
    {
        prepare();
    }

    ~Tx();

    void add(const fs::path &work, const fs::path &dst);

    const fs::path root() const { return root_; }

    fs::path tilesetPath(const std::string &tilesetId, bool tmp = false)
        const
    {
        return createPath(storage_paths::tilesetPath
                          (root_, tilesetId, tmp, tmpRoot_));
    }

    fs::path gluePath(const Glue &glue, bool tmp = false)
        const
    {
        return createPath(storage_paths::gluePath
                          (root_, glue, tmp, tmpRoot_));
    }

    fs::path virtualSurfacePath(const VirtualSurface &virtualSurface
                                , bool tmp = false)
        const
    {
        return createPath(storage_paths::virtualSurfacePath
                          (root_, virtualSurface, tmp, tmpRoot_));
    }

    TileSet open(const TilesetId &tilesetId) const;

    TileSet open(const Glue &glue) const;

    fs::path addGlue(const Glue &glue);

    fs::path addTileset(const std::string &tilesetId);

    fs::path addVirtualSurface(const VirtualSurface &virtualSurface);

    void remove(const fs::path &path);

    void commit();

    const GlueRule::list& glueRules() const { return glueRules_; }

private:
    void rollback();

    fs::path createPath(const fs::path &path) const;

    void prepare();

    const fs::path root_;
    const boost::optional<fs::path> &tmpRoot_;
    const OpenOptions openOptions_;

    typedef std::map<fs::path, fs::path> Mapping;
    Mapping mapping_;

    GlueRule::list glueRules_;
};

void Tx::prepare()
{
    glueRules_ = loadGlueRules(root_ / storage_paths::glueRulesPath(), true);
}

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
    return openTileSet(tilesetPath(tilesetId), openOptions_);
}

TileSet Tx::open(const Glue &glue) const
{
    return openTileSet(gluePath(glue), openOptions_);
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

fs::path Tx::addVirtualSurface(const VirtualSurface &virtualSurface)
{
    auto tmp(virtualSurfacePath(virtualSurface, true));
    add(tmp, virtualSurfacePath(virtualSurface));
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

TileSets openTilesets(Tx &tx, const StoredTileset::list &infos)
{
    TileSets tilesets;
    std::size_t index(0);
    for (const auto &info : infos) {
        tilesets.push_back(tx.open(info.tilesetId));
        LOG(info2) << "Opened tileset <" << info.tilesetId << ">.";
        ++index;
    }
    return tilesets;
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
    StoredTileset stored;

    /** Set of tilesets that overlap with this one.
     */
    std::set<std::size_t> incidentSets;

    Ts(int index, const TileSet &tileset, const LodRange &lodRange
       , const Storage::Properties &properties, bool added)
        : index(index), added(added), set(tileset)
        , stored(properties.tilesets[index])
        , lodRange(lodRange)
    {}

    bool notoverlaps(const Ts &other) const {
        return sphereOfInfluence().notoverlaps
            (other.sphereOfInfluence(), TileIndex::Flag::any);
    }

    std::string id() const { return stored.tilesetId; }

    std::string base() const { return stored.baseId; }

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
    std::size_t index;
    TileSet::list combination;
    Glue glue;
    TilesetId glueSetId;

    GlueDescriptor(std::size_t index, const TileSet::list &combination
                   , const Glue &glue, const TilesetId &glueSetId)
        : index(index), combination(combination), glue(glue)
        , glueSetId(glueSetId)
    {}

    typedef std::vector<GlueDescriptor> list;
};

GlueDescriptor::list prepareGlues(Tx &tx, Ts::list &tilesets, Ts &added)
{
    Ts::ptrlist incidentSets;
    {
        //save first for added tileset itself
        incidentSets.push_back(&added);

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
            // note that added tileset coincides with this
            incidentSets[0]->incidentSets.insert(ts.index);
        }
    }

    // for each tileset in the input
    {
        for (auto iincidentSets(incidentSets.begin())
                 , eincidentSets(incidentSets.end());
             iincidentSets != eincidentSets; ++iincidentSets)
        {
            auto &first(**iincidentSets);

            // process all remaining tilesets
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

    /* Build putative glues:
     * do a depth-first search where any two consecutive datasets are incident
     * this may yield some false positives ( A->B, A->C, B->C coincide does not
     * imply ABC coincide) but these will be ruled out in the next step.
     */

    GlueDescriptor::list gd;
    {
        // fwd
        std::function<void(const Ts&, Ts::const_ptrlist
                           , GlueRuleChecker, std::set<TilesetId>)>
            buildGlueCombination;

        // lambda
        buildGlueCombination = ([&]( const Ts& tsToAdd
            , Ts::const_ptrlist glueMembers
            , GlueRuleChecker ruleChecker
            , std::set<TilesetId> seenBases) -> void
        {
            LOG(info2) << "Trying tileset: " << tsToAdd.index << " <"
                       << tsToAdd.id() << "> ";
            if (!seenBases.insert(tsToAdd.base()).second) {
                // this base tileset has already been seen
                LOG(info1) << "Backtracking due to different version.";
                return;
            }

            if (!ruleChecker(tsToAdd.stored)) {
                // glue rule prevents glue generation
                LOG(info1) << "Backtracking due to rule check failed.";
                return;
            }

            // test if really incident with all already added
            for (auto & ats : glueMembers) {
                if (ats->incidentSets.find(tsToAdd.index)
                    == ats->incidentSets.end())
                {
                    LOG(info1)
                        << "Backtracking due to not incident with all "
                        "previous tilesets.";
                    return;
                }
            }

            // seems fine, add tileset
            glueMembers.push_back(&tsToAdd);

            if (glueMembers.size() > 1) {
                // sort glue content
                std::sort( glueMembers.begin(), glueMembers.end()
                         , [](const Ts* a, const Ts* b) {
                            return a->index < b->index;
                         } );

                Glue glue;
                TileSet::list combination;

                for (const auto gts : glueMembers) {
                    glue.id.push_back(gts->id());
                    combination.push_back(gts->set);
                }

                auto glueSetId(glueId2path(glue.id));
                glue.path = glueSetId;

                // create glue
                gd.emplace_back(gd.size() + 1, combination, glue, glueSetId);
            }

            // try tilesets incident with this tileset
            for ( auto its(tsToAdd.incidentSets.begin())
                ; its != tsToAdd.incidentSets.end(); ++its)
            {
                buildGlueCombination(tilesets[*its], glueMembers
                                     , ruleChecker, seenBases);
            }
        });

        buildGlueCombination(added, {}, GlueRuleChecker(tx.glueRules()), {});
    }

    // result
    return gd;
}

GlueDescriptor::list
prepareGlues(Tx &tx, Storage::Properties properties
             , const std::tuple<TileSets, std::size_t> &tsets)
{
    if (properties.tilesets.size() <= 1) {
        LOG(info3) << "No need to create any glue.";
        return {};
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
    auto gds(prepareGlues(tx, tilesets, tilesets[std::get<1>(tsets)]));

    // done
    return gds;
}

GlueDescriptor::list
prepareGlues(const Glue::list &glues, const TileSets &tilesets)
{
    GlueDescriptor::list gds;

    for (const auto &glue : glues) {
        TileSet::list combination;

        auto itilesets(tilesets.begin()), etilesets(tilesets.end());
        auto iglueId(glue.id.begin()), eglueId(glue.id.end());

        while ((itilesets != etilesets) && (iglueId != eglueId)) {
            const auto &ts(*itilesets);
            const auto tsId(ts.id());
            const auto &gtsId(*iglueId);

            if (tsId != gtsId) {
                ++itilesets;
                continue;
            }

            combination.push_back(ts);
            ++itilesets;
            ++iglueId;
        }

        utility::expect((iglueId == eglueId)
                        , "Cannot assign tilesets to glue <%s>"
                        , utility::join(glue.id, ","));

        auto glueSetId(glueId2path(glue.id));
        gds.emplace_back(gds.size() + 1, combination, glue, glueSetId);
    }

    // done
    return gds;
}

void writePendingGlues(Storage::Properties &properties
                       , const GlueDescriptor::list &gds)
{
    for (const auto &gd : gds) {
        properties.pendingGlues.insert(gd.glue.id);
    }
}

Glue createGlue(Tx &tx, const GlueDescriptor &gd
                , const Storage::AddOptions &addOptions
                , std::size_t glueCount)
{
    LOG(info3)
        << "Trying to generate glue #" << gd.index
        << '/' << glueCount << " <" << gd.glueSetId << ">.";
    vtslibs::storage::TIDGuard tg
        (str(boost::format("%d:%s") % gd.index % gd.glueSetId)
         , true);

    TileSetProperties gprop;
    gprop.id = gd.glueSetId;
    gprop.referenceFrame
        = gd.combination.front().getProperties().referenceFrame;

    reportMemoryUsage("before glue creation");

    // create glue
    auto gPath(tx.addGlue(gd.glue));
    auto gts(createTileSet(gPath, gprop, CreateMode::overwrite));

    // create glue
    utility::DurationMeter timer;
    TileSet::createGlue(gts, gd.combination, addOptions);
    auto duration(timer.duration());

    reportMemoryUsage("after merge");

    // empty cached input data (no need for them now)
    for (auto &ts : gd.combination) { ts.emptyCache(); }

    reportMemoryUsage("after emptying input caches");

    Glue glue(gd.glue);

    if (gts.empty()) {
        // unusable
        LOG(info3)
            << "Glue <" << gprop.id  << "> contains no tile; "
            << "ignoring. Duration: "
            << utility::formatDuration(duration) << ".";

        tx.remove(gPath);
        glue.path = {};
    } else {
        // usable
        LOG(info3)
            << "Glue <" << gprop.id  << "> created, duration: "
            << utility::formatDuration(duration) << ".";

        // flush
        gts.flush();

        reportMemoryUsage("after glue flush");
    }

    reportMemoryUsage("after glue creation");
    return glue;
}

} // namespace

std::tuple<Storage::Properties, StoredTileset>
Storage::Detail::addTileset(const Properties &properties
                            , const TilesetId &tilesetId
                            , const AddOptions &addOptions
                            , const Location &where) const
{
    if (tilesetId.find('@') != std::string::npos) {
        LOGTHROW(err1, vtslibs::storage::TileSetAlreadyExists)
            << "Invalid character in tileset ID <" << tilesetId << ">.";
    }

    StoredTileset tileset;
    {
        tileset.baseId = tilesetId;
        auto lastVersion(properties.lastVersion(tilesetId));
        if (lastVersion >= 0) {
            if (!addOptions.bumpVersion) {
                LOGTHROW(err1, vtslibs::storage::TileSetAlreadyExists)
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

        // assign tags
        tileset.tags = addOptions.tags;
    }

    if (properties.hasTileset(tileset.tilesetId)) {
        LOGTHROW(err1, vtslibs::storage::TileSetAlreadyExists)
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
        LOGTHROW(err1, vtslibs::storage::NoSuchTileSet)
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

std::tuple<Storage::Properties, Glue::map, VirtualSurface::map>
Storage::Detail::removeTilesets(const Properties &properties
                                , const TilesetIdList &tilesetIds)
    const
{
    std::tuple<Properties, Glue::map, VirtualSurface::map>
        res(properties, {}, {});
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
    auto &resGlues(std::get<1>(res));
    for (const auto &tilesetId : tilesetIds) {
        for (auto iglues(p.glues.begin()); iglues != p.glues.end(); ) {
            if (iglues->second.references(tilesetId)) {
                resGlues.insert(*iglues);
                iglues = p.glues.erase(iglues);
            } else {
                ++iglues;
            }
        }

        for (auto iglues(p.pendingGlues.begin());
             iglues != p.pendingGlues.end(); )
        {
            if (Glue::references(*iglues, tilesetId)) {
                iglues = p.pendingGlues.erase(iglues);
            } else {
                ++iglues;
            }
        }

        for (auto iglues(p.emptyGlues.begin());
             iglues != p.emptyGlues.end(); )
        {
            if (Glue::references(*iglues, tilesetId)) {
                iglues = p.emptyGlues.erase(iglues);
            } else {
                ++iglues;
            }
        }
    }

    auto &virtualSurfaces(p.virtualSurfaces);
    auto &resVirtualSurfaces(std::get<2>(res));
    for (const auto &tilesetId : tilesetIds) {
        for (auto ivirtualSurfaces(virtualSurfaces.begin());
             ivirtualSurfaces != virtualSurfaces.end(); )
        {
            if (ivirtualSurfaces->second.references(tilesetId)) {
                resVirtualSurfaces.insert(*ivirtualSurfaces);
                ivirtualSurfaces = virtualSurfaces.erase(ivirtualSurfaces);
            } else {
                ++ivirtualSurfaces;
            }
        }
    }

    return res;
}

std::tuple<Storage::Properties, VirtualSurface::map>
Storage::Detail::removeVirtualSurfaces(const Properties &properties
                                       , const VirtualSurface::Ids &ids)
    const
{
    std::tuple<Properties, VirtualSurface::map> res(properties, {});
    auto &p(std::get<0>(res));
    auto &resVirtualSurfaces(std::get<1>(res));

    for (const auto &id : ids) {
        auto fvirtualSurfaces(p.findVirtualSurface(id));
        if (fvirtualSurfaces == p.virtualSurfaces.end()) {
            LOG(warn2)
                << "Virtual surface <"
                << utility::join(id, ",") << "> "
                "not found in storage " << root << ".";
        } else {
            resVirtualSurfaces.insert(*fvirtualSurfaces);
            p.virtualSurfaces.erase(fvirtualSurfaces);
        }
    }

    return res;
}

void generateGluesImpl(Tx &tx, const GlueDescriptor::list &gds
                       , Storage::Detail &detail
                       , Storage::Properties &properties
                       , const Storage::AddOptions &addOptions
                       , ScopedStorageLock &storageLock)
{
    // report
    LOG(info3) << "Generating " << gds.size() << " glue(s):";
    for (const auto &gd : gds) {
        LOG(info3)
            << "    #" << gd.index
            << '/' << gds.size() << " <" << gd.glueSetId << ">.";
    }

    // not a lazy add, try to generate all glues

    // prepare progress if available
    if (addOptions.progress) {
        // accumulate total number of tiles to generate
        std::size_t total(0);
        for (const auto &gd : gds) {
            total += TileSet::analyzeGlue(gd.combination, addOptions)
                .tilesToGenerate;
        }
        // notify progress about how many tiles to expect
        addOptions.progress->expect(total);
    }

    // run the thing
    for (const auto &gd : gds) {
        // create glue

        Glue glue;
        {
            // create glue under glue lock with unlocked storage
            ScopedStorageLock glueLock
                (addOptions.locker, lockName(gd.glue), &storageLock);
            glue = createGlue(tx, gd, addOptions, gds.size());
        }

        // update properties
        if (glue.path.empty()) {
            // empty glue -> move to empty list
            properties.pendingGlues.erase(glue.id);
            properties.emptyGlues.insert(glue.id);
        } else {
            // create -> move to main map
            properties.pendingGlues.erase(glue.id);
            properties.glues[glue.id] = glue;
        }

        // lazy add?
        if (addOptions.mode != Storage::AddOptions::Mode::legacy) {
            // TODO: load storage config again
            // commit new properties and changes to the transaction
            detail.saveConfig(properties);
            tx.commit();
        }
    }
}

void Storage::Detail::add(const TileSet &tileset, const Location &where
                          , const TilesetId &tilesetId
                          , const AddOptions &addOptions)
{
    // LOCKING: lock whole storage
    ScopedStorageLock storageLock(addOptions.locker);

    std::string simulation(addOptions.dryRun ? "(simulation) " : "");
    vtslibs::storage::TIDGuard tg
        (str(boost::format("%sadd(%s)") % simulation % tilesetId));

    // check compatibility
    if (tileset.getProperties().referenceFrame != properties.referenceFrame) {
        LOGTHROW(err1, vtslibs::storage::IncompatibleTileSet)
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
        = addTileset(properties, tilesetId, addOptions, where);

    LOG(info3)
        << "Adding tileset <" << tileset.id() << "> (from "
        << tileset.root() << ").";

    Tx tx(root, addOptions.tmp, addOptions.openOptions);

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
                            .lodRange(addOptions.filter.lodRange())
                                .openOptions(addOptions.openOptions)
                            );
    }());

    auto tilesets(openTilesets(tx, nProperties.tilesets
                               , dst, tilesetInfo.tilesetId));

    auto gds(prepareGlues(tx, nProperties, tilesets));

    // dry run -> do nothing
    if (addOptions.dryRun) { return; }

    writePendingGlues(nProperties, gds);

    if (addOptions.mode != AddOptions::Mode::legacy) {
        // new interface: commit new properties and changes to transaction
        saveConfig(nProperties);
        tx.commit();
    }

    // lazy add? wrap it here
    if (addOptions.mode == AddOptions::Mode::lazy) { return; }

    // generate all glue
    generateGluesImpl(tx, gds, *this, nProperties, addOptions
                      , storageLock);

    if (addOptions.mode == AddOptions::Mode::legacy) {
        // old interface: flush

        // LOCKING TODO: reload config and merge

        saveConfig(nProperties);
        tx.commit();
    }
}

void Storage::Detail::generateGlues(const TilesetId &tilesetId
                                    , const AddOptions &addOptions)
{
    // LOCKING: lock whole storage
    ScopedStorageLock storageLock(addOptions.locker);

    Glue::list glues;

    for (const auto &id : properties.pendingGlues) {
        if (std::find(id.begin(), id.end(), tilesetId) != id.end()) {
            glues.emplace_back(id, glueId2path(id));
        }
    }

    if (glues.empty()) {
        LOG(info3) << "All glues for tileset <" << tilesetId << "> exist.";
        return;
    }

    Tx tx(root, addOptions.tmp, addOptions.openOptions);

    const auto gds(prepareGlues(glues, openTilesets(tx, properties.tilesets)));

    // generate all glues
    generateGluesImpl(tx, gds, *this, properties, addOptions, storageLock);
}

void Storage::Detail::generateGlue(const Glue::Id &glueId
                                   , const AddOptions &addOptions)
{
    // LOCKING: lock whole storage
    ScopedStorageLock storageLock(addOptions.locker);

    bool overwrite(false);

    if (properties.hasGlue(glueId)) {
        if (addOptions.overwrite) {
            overwrite = true;
            LOG(info3)
                << "Re-generating existing glue <"
                << utility::join(glueId, ",") << ">.";
        } else {
            LOGTHROW(err3, std::runtime_error)
                << "Glue <" << utility::join(glueId, ",")
                << "> already exists.";
        }
    }

    if (properties.hasEmptyGlue((glueId))) {
        if (!overwrite) {
            LOGTHROW(err3, std::runtime_error)
                << "Glue <" << utility::join(glueId, ",") << "> is empty.";
        }
    }

    if (!properties.hasPendingGlue((glueId))) {
        if (!overwrite) {
            LOGTHROW(err3, std::runtime_error)
                << "Glue <" << utility::join(glueId, ",") << "> not found.";
        }
    }

    Glue::list glues;
    glues.emplace_back(glueId, glueId2path(glueId));

    Tx tx(root, addOptions.tmp, addOptions.openOptions);

    const auto gds
        (prepareGlues(glues, openTilesets(tx, properties.tilesets)));

    // generate all glues
    generateGluesImpl(tx, gds, *this, properties, addOptions, storageLock);
}

void Storage::Detail::remove(const TilesetIdList &tilesetIds
                             , const StorageLocker::pointer &locker)
{
    Properties nProperties;
    Glue::map glues;
    VirtualSurface::map virtualSurfaces;
    std::tie(nProperties, glues, virtualSurfaces)
        = removeTilesets(properties, tilesetIds);

    LOG(info3)
        << "Removing tilesets <" << utility::join(tilesetIds, ", ") << ">.";

    // LOCKING: lock whole storage
    ScopedStorageLock storageLock(locker);
    properties = nProperties;
    saveConfig();

    // physical removal
    for (const auto &tilesetId : tilesetIds) {
        auto path(storage_paths::tilesetPath(root, tilesetId));
        LOG(info3) << "Removing tileset <" << tilesetId
                   << "> from " << path << ".";
        rmrf(path);
    }

    for (const auto &item : glues) {
        const auto &glue(item.second);
        auto path(storage_paths::gluePath(root, glue));
        LOG(info3) << "Removing glue <" << utility::join(glue.id, ",")
                   << "> from " << glue.path << ".";
        rmrf(path);
    }

    for (const auto &item : virtualSurfaces) {
        const auto &virtualSurface(item.second);
        auto path(storage_paths::virtualSurfacePath(root, virtualSurface));
        LOG(info3) << "Removing virtual surface <"
                   << utility::join(virtualSurface.id, ",")
                   << "> from " << virtualSurface.path << ".";
        rmrf(path);
    }
}

void Storage::Detail
::createVirtualSurface(const TilesetIdSet &tilesets
                       , CreateMode mode
                       , const StorageLocker::pointer &locker)
{
    // LOCKING: lock whole storage
    ScopedStorageLock storageLock(locker);

    auto tmp(tilesets);

    VirtualSurface vs;
    for (const auto &stored : properties.tilesets) {
        auto ftmp(tmp.find(stored.tilesetId));
        if (ftmp == tmp.end()) { continue; }

        vs.id.push_back(stored.tilesetId);
        tmp.erase(ftmp);
    }

    if (!tmp.empty()) {
        LOGTHROW(err1, vtslibs::storage::NoSuchTileSet)
            << "Tileset(S) <" << utility::join(tmp, ", ")
            << "> not found in storage " << root << ".";
    }

    if ((mode == CreateMode::failIfExists)
        && properties.getVirtualSurface(vs.id))
    {
        LOGTHROW(err1, vtslibs::storage::TileSetAlreadyExists)
            << "Virtual surface <" << utility::join(vs.id, ",")
            << "> already present in storage "
            << root << ".";
    }

    LOG(info3) << "Creating virtual surface <" << utility::join(vs.id, ",")
               << "> in storage " << root << ".";

    const auto vsSetId(boost::lexical_cast<std::string>
                       (utility::join(vs.id, "_")));
    vs.path = vsSetId;

    auto nProperties(properties);
    {
        // LOCKING: lock virtual surface
        ScopedStorageLock vsLock(locker, lockName(vs));
        // LOCKING: and unlock storage
        storageLock.unlock();

        Tx tx(root, boost::none);

        auto vsPath(tx.addVirtualSurface(vs));

        vts::aggregateTileSets
            (vsPath, "../.."
             , CloneOptions()
             .mode(CreateMode::overwrite)
             .tilesetId(vsSetId)
             .createFlags(AggreateFlags::dontAbsolutize
                          | AggreateFlags::sourceReferencesInMetatiles)
             , TilesetIdSet(vs.id.begin(), vs.id.end()));

        nProperties.virtualSurfaces[vs.id] = vs;

        tx.commit();
    }

    // LOCKING: lock whole storage again
    storageLock.lock();

    // TODO: LOCKING: reload properties and check virtual surface validity

    // commit properties
    properties = nProperties;
    saveConfig();
}

void Storage::Detail
::removeVirtualSurface(const TilesetIdSet &tilesets
                       , const StorageLocker::pointer &locker)
{
    // LOCKING: lock whole storage
    ScopedStorageLock storageLock(locker);

    auto tmp(tilesets);

    VirtualSurface::Id vsId;
    for (const auto &stored : properties.tilesets) {
        auto ftmp(tmp.find(stored.tilesetId));
        if (ftmp == tmp.end()) { continue; }

        vsId.push_back(stored.tilesetId);
        tmp.erase(ftmp);
    }

    if (!tmp.empty()) {
        LOGTHROW(err1, vtslibs::storage::NoSuchTileSet)
            << "Tileset(S) <" << utility::join(tmp, ", ")
            << "> not found in storage " << root << ".";
    }

    Properties nProperties;
    VirtualSurface::map virtualSurfaces;
    std::tie(nProperties, virtualSurfaces)
        = removeVirtualSurfaces(properties, { vsId });

    properties = nProperties;
    saveConfig();

    for (const auto &item : virtualSurfaces) {
        const auto &virtualSurface(item.second);
        auto path(storage_paths::virtualSurfacePath(root, virtualSurface));
        LOG(info3)
            << "Removing virtual Surface " << virtualSurface.path << ".";
        rmrf(path);
    }
}

} } // namespace vtslibs::vts
