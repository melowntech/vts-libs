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

#include "../../storage/error.hpp"
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

void Storage::add(const boost::filesystem::path &tilesetPath
                  , const Location &where
                  , const StoredTileset &info
                  , const TileFilter &filter)
{
    auto ts(openTileSet(tilesetPath));
    StoredTileset useInfo(info);
    if (useInfo.tilesetId.empty()) {
        useInfo.tilesetId = ts.getProperties().id;
    }
    detail().add(ts, where, useInfo, filter);
}

void Storage::remove(const TilesetIdList &tilesetIds)
{
    detail().remove(tilesetIds);
}

TileSet Storage::flatten(const boost::filesystem::path &tilesetPath
                         , CreateMode mode
                         , const boost::optional<std::string> tilesetId)
{
    return detail()
        .flatten(tilesetPath, mode
                 , (tilesetId ? *tilesetId
                    : tilesetPath.filename().string()));
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
openTilesets(Tx &tx, const StoredTileset::list &infos, TileSet &tileset
             , const boost::optional<StoredTileset::GlueMode> &glueMode
             = boost::none)
{
    std::tuple<TileSets, std::size_t> res;
    TileSets &tilesets(std::get<0>(res));
    std::size_t index(0);
    for (const auto &info : infos) {
        if (glueMode && (*glueMode != info.glueMode)) {
            LOG(info2) << "Skipping masked out tileset <"
                       << info.tilesetId << ">.";
            continue;
        }
        if (info.tilesetId == tileset.id()) {
            tilesets.push_back(tileset);
            std::get<1>(res) = index;
            LOG(info2) << "Reused already open <"
                       << tilesets.back().id() << ">.";
        } else {
            tilesets.push_back(tx.open(info.tilesetId));
            LOG(info2) << "Opened tileset <" << tilesets.back().id() << ">.";
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

    /** Set of tilesets that overlap with this one.
     */
    std::set<std::size_t> incidentSets;

    Ts(const TileSet &tileset, const LodRange &lodRange
       , bool added)
        : index(0), added(added), set(tileset), lodRange(lodRange)
    {}

    bool notoverlaps(const Ts &other) const {
        if (!check(set.detail().properties.spatialDivisionExtents
                   , other.set.detail().properties.spatialDivisionExtents))
        {
            return true;
        }

        return sphereOfInfluence().notoverlaps
            (other.sphereOfInfluence(), TileIndex::Flag::any);
    }

    std::string id() const { return set.id(); }

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

Storage::Properties
createGlues(Tx &tx, Storage::Properties properties
            , const std::tuple<TileSets, std::size_t> &tsets)
{
    if (properties.tilesets.size() <= 1) {
        LOG(info3) << "No need to create any glue.";
        return properties;
    }

    // accumulate lod range for all tilesets
    auto lr(range(std::get<0>(tsets)));

    LOG(info1) << "Glue lod range: " << lr;

    // create tileset list
    Ts::list tilesets;
    {
        std::size_t index(0);
        for (auto &set : std::get<0>(tsets)) {
            tilesets.emplace_back(set, lr, (index == std::get<1>(tsets)));
            tilesets.back().index = index++;
        }
    }

    // filter out all tilesets that do not overlap with added tileset
    const auto &added(tilesets[std::get<1>(tsets)]);
    Ts::ptrlist incidentSets;
    {
        for (auto &ts : tilesets) {
            // ignore added tileset
            if (ts.added) {continue; }
            if (ts.notoverlaps(added)) { continue; }

            // incidence between spheres of influence -> remember
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

    for (const auto &tsp : incidentSets) {
        const auto &ts(*tsp);

        LOG(info4)
            << "Sets <" << added.id() << "> and <"
            << ts.id() << "> (" << added.index << ", " << ts.index
            << ") overlap with [" << utility::join(ts.incidentSets, ", ")
            << "].";

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
            // make room for combination
            TileSet::const_ptrlist combination;
            Glue glue;

            for (std::size_t index(0), e(mapping.size()); index != e; ++ index)
            {
                auto value(mapping[index]);
                switch (value) {
                case emptyPlaceholder:
                    continue;

                case addedPlaceholder:
                    combination.push_back(&added.set);
                    glue.id.push_back(added.id());
                    continue;

                case thisPlaceholder:
                    combination.push_back(&tsp->set);
                    glue.id.push_back(tsp->id());
                    continue;

                }

                if (flags[value]) {
                    combination.push_back(&tilesets[index].set);
                    glue.id.push_back(tilesets[index].id());
                }
            }

            // create glue
            auto glueSetId(boost::lexical_cast<std::string>
                           (utility::join(glue.id, "_")));
            glue.path = glueSetId;
            LOG(info3) << "Trying to generate glue <" << glueSetId << ">.";

            TileSetProperties gprop;
            gprop.id = glueSetId;
            gprop.referenceFrame
                = combination.front()->getProperties().referenceFrame;

            {
                // create glue
                auto tmpPath(tx.addGlue(glue));
                auto gts(createTileSet(tmpPath, gprop
                                       , CreateMode::overwrite));

                gts.createGlue(combination);

                if (gts.empty()) {
                    // unusable
                    LOG(info3)
                        << "Glue <" << glueSetId  << "> contains no tile; "
                        << "glue is forgotten.";
                    tx.remove(tmpPath);
                } else {
                    // usable
                    LOG(info3) << "Glue <" << glueSetId  << "> created.";

                    // flush
                    gts.flush();

                    // and remember
                    properties.glues[glue.id] = glue;
                }
            }

        } while (flags.increment());
    }

    return properties;
}

} // namespace

Storage::Properties Storage::Detail::addTileset(const Properties &properties
                                                , const StoredTileset &tileset
                                                , const Location &where) const
{
    if (properties.hasTileset(tileset.tilesetId)) {
        LOGTHROW(err1, vadstena::storage::TileSetAlreadyExists)
            << "Tileset <" << tileset.tilesetId
            << "> already present in storage "
            << root << ".";
    }

    auto p(properties);
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
        return p;
    }

    // some reference
    auto ftilesets(p.findTilesetIt(where.where));
    if (ftilesets == tilesets.end()) {
        LOGTHROW(err1, vadstena::storage::NoSuchTileSet)
            << "Tileset <" << tileset.tilesetId << "> (used as a reference) "
            "not found in storage " << root << ".";
    }

    if (where.direction == Location::Direction::below) {
        // below given reference -> just insert here
        tilesets.insert(ftilesets, tileset);
    } else {
        // above reference -> advance and insert
        tilesets.insert(std::next(ftilesets), tileset);
    }

    return p;
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

void Storage::Detail::add(const TileSet &tileset
                          , const Location &where
                          , const StoredTileset &tilesetInfo
                          , const TileFilter &filter)
{
    dbglog::thread_id(str(boost::format("%s->%s/%s")
                          % tileset.id()
                          % root.filename().string()
                          % tilesetInfo.tilesetId));

    // check compatibility
    if (tileset.getProperties().referenceFrame != properties.referenceFrame) {
        LOGTHROW(err1, vadstena::storage::IncompatibleTileSet)
            << "Tileset <" << tilesetInfo.tilesetId << "> "
            "uses different reference frame ("
            << tileset.getProperties().referenceFrame
            << ") from the one  this storage supports ("
            << properties.referenceFrame << ").";
    }

    // prepare new tileset list
    auto nProperties(addTileset(properties, tilesetInfo, where));

    LOG(info3) << "Adding tileset <" << tileset.id() << "> (from "
               << tileset.root() << ").";

    {
        Tx tx(root);

        // create tileset at work path (overwrite any existing stuff here)
        auto dst(cloneTileSet(tx.addTileset(tilesetInfo.tilesetId), tileset,
                              CloneOptions()
                              .mode(CreateMode::overwrite)
                              .tilesetId(tilesetInfo.tilesetId)
                              .lodRange(filter.lodRange())));

        // create glues only if tileset participates in any glue
        if (tilesetInfo.glueMode != StoredTileset::GlueMode::none) {
            auto tilesets(openTilesets(tx, nProperties.tilesets, dst
                                       , StoredTileset::GlueMode::full));
            nProperties = createGlues(tx, nProperties, tilesets);
        }

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

namespace {

typedef std::map<TilesetId, Glue::list> GlueMapping;

class Flattener : public Encoder {
public:
    struct Ts {
        TileSet set;
        TileIndex soi;

        Ts(Tx &tx, const TilesetId &id);
        Ts(TileSet &&set) : set(set) {}

        void generateSoi(const LodRange &lodRange) {
            soi = set.sphereOfInfluence(lodRange, TileIndex::Flag::mesh);
        }

        TilesetId id() const { return set.id(); }

        typedef std::vector<Ts> list;
        typedef std::vector<const Ts*> const_ptrlist;
    };

    struct GlueTs : Ts {
        GlueTs(Tx &tx, const Glue &glue);

        Glue::Id id;

        typedef std::vector<GlueTs> list;
        typedef std::map<TilesetId, list> map;
    };

    Flattener(Tx &tx, const Storage::Properties &properties
              , const boost::filesystem::path &path
              , const TileSetProperties &tsProps
              , CreateMode mode)
        : Encoder(path, tsProps, mode), tx_(tx)
        , properties_(properties)
        , tilesets_(openTilesets(tx, properties.tilesets))
        , dataLodRange_(range(tilesets_))
        , soi_(soi(dataLodRange_.max, tilesets_))
        , glues_(openGlues(tx, dataLodRange_.max, properties.glues))
    {
        // limit lodRange
        setConstraints(Constraints()
                       .setLodRange(dataLodRange_)
                       .setValidTree(&soi_));
    }

private:
    virtual TileResult
    generate(const TileId &tileId, const NodeInfo &nodeInfo
             , const TileResult&);

    virtual void finish(TileSet &tileSet);

    GlueMapping getGluesMapping(const Glue::Id &glueId) const;

    const GlueTs::list* glues(const TilesetId &tilesetId) const {
        auto fglues(glues_.find(tilesetId));
        if (fglues == glues_.end()) { return nullptr; }
        return &fglues->second;
    }

    bool trySet(TileResult &result, const TileId &tileId, const Ts &ts)
    {
        if (!ts.set.exists(tileId)) { return false;; }

        UTILITY_OMP(critical)
        {
            // this must be inside critical section
            result.source() = ts.set.getTileSource(tileId);
            usedSets_.insert(&ts.set);
        }
        return true;
    }

    static Ts::list openTilesets(Tx &tx, const StoredTileset::list &infos);

    static GlueTs::map openGlues(Tx &tx, Lod depth, const Glue::map &glues);

    static TileIndex soi(Lod depth, Ts::list &sets);

    static LodRange range(const Ts::list &tilesets);

    Tx &tx_;
    const Storage::Properties properties_;
    Ts::list tilesets_;
    const LodRange dataLodRange_;
    const TileIndex soi_;
    GlueTs::map glues_;

    // accumulates used sets
    std::set<const TileSet*> usedSets_;
};

LodRange Flattener::range(const Ts::list &tilesets)
{
    auto lr(LodRange::emptyRange());
    for (const auto &ts : tilesets) {
        lr = unite(lr, ts.set.lodRange());
    }
    return lr;
}

Flattener::Ts::Ts(Tx &tx, const TilesetId &id)
    : set(tx.open(id))
{}

Flattener::GlueTs::GlueTs(Tx &tx, const Glue &glue)
    : Ts(tx.open(glue)), id(glue.id)
{}

Flattener::Ts::list
Flattener::openTilesets(Tx &tx, const StoredTileset::list &infos)
{
    Ts::list sets;

    for (const auto &info : infos) {
        sets.emplace_back(tx, info.tilesetId);
        LOG(info2) << "Opened tileset <" << sets.back().set.id() << ">.";
    }

    return sets;
}

Flattener::GlueTs::map Flattener::openGlues(Tx &tx, Lod depth
                                            , const Glue::map &glues)
{
    GlueTs::map gm;
    LodRange lr(0, depth);

    for (const auto &g : glues) {
        auto &list(gm[g.first.back()]);
        list.emplace_back(tx, g.second);
        list.back().generateSoi(lr);
        LOG(info2)
            << "Opened glue <" << utility::join(list.back().id, ",") << ">.";
    }

    return gm;
}

TileIndex Flattener::soi(Lod depth, Ts::list &sets)
{
    // get sphere of inlfuence of all meshes
    LodRange lr(0, depth);
    vts::TileIndices sois;
    for (auto &ts : sets) {
        ts.generateSoi(lr);
        sois.push_back(&ts.soi);
    }

    // sphere of influence of whole storage
    return unite(sois, TileIndex::Flag::any, lr);
}

Encoder::TileResult
Flattener::generate(const TileId &tileId, const NodeInfo &nodeInfo
                    , const TileResult&)
{
    Ts::const_ptrlist sets;
    Glue::Id glueId;

    // get list of tilesets that can be source of given tile
    for (const auto &ts : tilesets_) {
        if (ts.soi.get(tileId)) {
            sets.push_back(&ts);
            glueId.push_back(ts.set.id());
        }
    }

    TileResult result;

    for (const auto &ts : boost::adaptors::reverse(tilesets_)) {
        if (const auto *gs = glues(ts.id())) {
            // TODO: check only appropriate glues (i.e. those that correspond
            // with glueId constructed above
            for (const auto &glue : *gs) {
                if (trySet(result, tileId, glue)) { return result; }
            }
        }

        if (trySet(result, tileId, ts)) { return result; }
    }

    // nothing appropriate
    return result;

    (void) nodeInfo;
}

void Flattener::finish(TileSet &tileSet)
{
    for (const auto *set : usedSets_) {
        const auto props(set->getProperties());
        tileSet.addCredits(props.credits);
        tileSet.addBoundLayers(props.boundLayers);
    }

    if (!usedSets_.empty()) {
        tileSet.setPosition((*usedSets_.begin())->getProperties().position);
    }
}

} // namespace

TileSet Storage::Detail::flatten(const boost::filesystem::path &tilesetPath
                                 , CreateMode mode
                                 , const std::string &tilesetId)
{
    TileSetProperties tsProp;
    tsProp.id = tilesetId;
    tsProp.referenceFrame = properties.referenceFrame;

    Tx tx(root);
    Flattener flattener(tx, properties, tilesetPath, tsProp, mode);

    return flattener.run();
}

} } // namespace vadstena::vts
