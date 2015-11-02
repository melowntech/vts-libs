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
#include "utility/guarded-call.hpp"

#include "../../storage/error.hpp"
#include "../storage.hpp"
#include "../../vts.hpp"
#include "./detail.hpp"
#include "../tileset/detail.hpp"

#include "./config.hpp"
#include "./paths.hpp"

namespace fs = boost::filesystem;

namespace vadstena { namespace vts {

void Storage::add(const boost::filesystem::path &tilesetPath
                  , const Location &where
                  , const boost::optional<std::string> tilesetId)
{
    auto ts(openTileSet(tilesetPath));
    detail().add(ts, where
                 , tilesetId ? *tilesetId : ts.getProperties().id);
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

    TileSet open(const std::string &tilesetId) const;

    fs::path addGlue(const Glue &glue);

    fs::path addTileset(const std::string &tilesetId);

    void remove(const fs::path &path);

private:
    void rollback();
    void commit();

    fs::path createPath(const fs::path &path) const;

    const fs::path root_;
    typedef std::map<fs::path, fs::path> Mapping;
    Mapping mapping_;
};

Tx::~Tx() {
    if (std::uncaught_exception()) {
        // we cannot throw!
        rollback();
    } else {
        commit();
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
}

TileSet Tx::open(const std::string &tilesetId) const
{
    return openTileSet(tilesetPath(tilesetId));
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
openTilesets(Tx &tx, const TilesetIdList &ids, TileSet &tileset)
{
    std::tuple<TileSets, std::size_t> res;
    TileSets &tilesets(std::get<0>(res));
    std::size_t index(0);
    for (const auto &id : ids) {
        if (id == tileset.id()) {
            tilesets.push_back(tileset);
            std::get<1>(res) = index;
            LOG(info2) << "Reused <" << tilesets.back().id() << ">.";
        } else {
            tilesets.push_back(tx.open(id));
            LOG(info2) << "Opened <" << tilesets.back().id() << ">.";
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

    LOG(info4) << lr;

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

                if (false && gts.empty()) {
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
                                                , const std::string tilesetId
                                                , const Location &where) const
{
    if (properties.hasTileset(tilesetId)) {
        LOGTHROW(err1, vadstena::storage::TileSetAlreadyExists)
            << "Tileset <" << tilesetId << "> already present in storage "
            << root << ".";
    }

    auto p(properties);
    auto &tilesets(p.tilesets);

    if (where.where.empty()) {
        // void reference
        if (where.direction == Location::Direction::below) {
            // below void -> to the top of the stack
            tilesets.push_back(tilesetId);
        } else {
            // above void -> to the bottom of the stack
            tilesets.insert(tilesets.begin(), tilesetId);
        }
        return p;
    }

    // some reference
    auto ftilesets(p.findTileset(where.where));
    if (ftilesets == tilesets.end()) {
        LOGTHROW(err1, vadstena::storage::NoSuchTileSet)
            << "Tileset <" << tilesetId << "> (used as a reference) "
            "not found in storage " << root << ".";
    }

    if (where.direction == Location::Direction::below) {
        // below given reference -> just insert here
        tilesets.insert(ftilesets, tilesetId);
    } else {
        // above reference -> advance and insert
        tilesets.insert(std::next(ftilesets), tilesetId);
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
        auto ftilesets(p.findTileset(tilesetId));
        if (ftilesets == tilesets.end()) {
            LOG(warn1) << "Tileset <" << tilesetId << "> "
                "not found in storage " << root << ".";
            continue;
        }

        // remove from tilesets
        tilesets.erase(ftilesets);
    }

    // drop all glues that reference requested tilesets
    auto &resGlues(std::get<1>(res));
    for (const auto &tilesetId : tilesetIds) {
        for (auto iresGlues(resGlues.begin()); iresGlues != resGlues.end(); ) {
            if (iresGlues->second.references(tilesetId)) {
                iresGlues = resGlues.erase(iresGlues);
            } else {
                ++iresGlues;
            }
        }
    }

    return res;
}

void Storage::Detail::add(const TileSet &tileset
                          , const Location &where
                          , const std::string tilesetId)
{
    dbglog::thread_id(str(boost::format("%s->%s/%s")
                          % tileset.id()
                          % root.filename().string()
                          % tilesetId));

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
    auto nProperties(addTileset(properties, tilesetId, where));

    LOG(info3) << "Adding tileset <" << tileset.id() << "> (from "
               << tileset.root() << ").";

    {
        Tx tx(root);

        // create tileset at work path (overwrite any existing stuff here)
        auto dst(cloneTileSet(tx.addTileset(tilesetId), tileset,
                              CloneOptions()
                              .mode(CreateMode::overwrite)
                              .tilesetId(tilesetId)));

        // create glues
        auto tilesets(openTilesets(tx, nProperties.tilesets, dst));

        nProperties = createGlues(tx, nProperties, tilesets);
    }

    // FIXME: remove
    if (::getenv("ABORT")) {
        exit(-1);
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
