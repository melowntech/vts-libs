#include <iterator>

#include <boost/format.hpp>
#include <boost/range/sub_range.hpp>

#include "utility/progress.hpp"
#include "utility/path.hpp"

#include "../io.hpp"

#include "../../storage/tidguard.hpp"
#include "./detail.hpp"
#include "./merge.hpp"

namespace fs = boost::filesystem;

namespace vadstena { namespace vts {

namespace {

typedef std::vector<TileIndex> TileIndexList;

inline TileIndexList
tileIndices(const TileSet::const_ptrlist &sets
            , const LodRange &lodRange
            , std::size_t size
            = std::numeric_limits<std::size_t>::max())
{
    size = std::min(size, sets.size());

    std::vector<TileIndex> indices;
    auto isets(sets.begin());
    for (std::size_t i(0); i < size; ++i) {
        indices.push_back
            ((*isets)->tileIndex(lodRange).simplify(TileIndex::Flag::mesh));
        ++isets;
    }
    return indices;
}

LodRange range(const TileSet::const_ptrlist &sets)
{
    // start with invalid range
    LodRange r(LodRange::emptyRange());

    if (sets.empty()) { return r; }

    for (const auto &set : sets) {
        r = unite(r, set->lodRange());
    }
    return r;
}

const char *TILEINDEX_DUMP_ROOT("TILEINDEX_DUMP_ROOT");

const char* getDumpDir()
{
    return std::getenv(TILEINDEX_DUMP_ROOT);
}

void dumpTileIndex(const char *root, const fs::path &name
                   , const TileIndex &index
                   , const std::string &referenceFrame)
{
    if (!root) { return; }
    LOG(info1) << "Dumping tileindex " << name << ".";
    auto filename(root / utility::addExtension(name, ".index"));
    create_directories(filename.parent_path());
    index.save(filename);

    auto rfFilename(root / utility::addExtension(name, ".rframe"));
    utility::write(rfFilename, referenceFrame.data(), referenceFrame.size());
}

inline void dump(const char *root, const boost::filesystem::path &dir
                 , const std::vector<TileIndex> &tileIndices
                 , const std::string &referenceFrame)
{
    if (!root) { return; }

    int i(0);
    for (const auto &ti : tileIndices) {
        dumpTileIndex(root, dir / str(boost::format("%03d") % i), ti
                      , referenceFrame);
        ++i;
    }
}

typedef std::vector<const TileSet::Detail*> DetailList;

DetailList details(TileSet::Detail &detail
                   , const TileSet::const_ptrlist &sets)
{
    DetailList out;
    for (const auto *set : sets) {
        out.push_back(&detail.other(*set));
    }
    return out;
}

struct Merger {
public:
    Merger(TileSet::Detail &self, const TileIndex &generate
           , const TileSet::const_ptrlist &srcSets)
        : self_(self), world_(generate), generate_(generate)
        , src_(details(self, srcSets)), top_(*src_.back())
        , topId_(src_.size() - 1), progress_(generate_.count())
    {
        // make world complete
        world_.complete();

        // ruin
        mergeTile(NodeInfo(self_.referenceFrame));
    }

private:
    struct Constraints : merge::MergeConstraints {
        Constraints(const Merger &merger, bool generable)
            : merge::MergeConstraints(generable), merger(merger)
        {}
        virtual bool feasible(const merge::Output &result) const {
            return merger.isGlueTile(result);
        }
        const Merger &merger;
    };
    friend struct Constraints;

    /** Merge subtree starting at index.
     *  Calls itself recursively.
     */
    void mergeTile(const NodeInfo &nodeInfo
                   , const TileId &tileId = TileId()
                   , const merge::Input::list &parentSource
                   = merge::Input::list());

    /** Generates new tile as a merge of tiles from other tilesets.
     */
    merge::Output processTile(const NodeInfo &nodeInfo
                              , const TileId &tileId
                              , const merge::Input::list &parentSource
                              , const Constraints &constraints);

    bool isGlueTile(const merge::Output &tile) const;

    TileSet::Detail &self_;
    TileIndex world_;
    const TileIndex &generate_;
    const DetailList src_;
    const TileSet::Detail &top_;
    const merge::Input::Id topId_;

    utility::Progress progress_;
};

inline bool Merger::isGlueTile(const merge::Output &tile) const
{
    // shortcut
    if (top_.fullyCovered(tile.tileId)) {
        LOG(info1) << "Fully covered by top set.";
        return false;
    }

    auto size(tile.source.size());
    auto srcSize(src_.size());

    // sanity check
    if (!size) { return false; }

    // special case
    if (tile.source.front().id() == topId_) {
        // generated only from top set, must be derived tile
        return tile.derived(0);
    }

    // if tile is generated from all input sets -> glue
    if (size == srcSize) {
        return true;
    }

    if ((size + 1) == srcSize) {
        // possibly generated from rest sets
        if (tile.source.back().id() == topId_) {
            // contains top -> no way
            return false;
        }

        // tile must be fully derived from other sets (such tile cannot exist in
        // others)
        return tile.fullyDerived();
    }

    // anything else -> not glue
    return false;
}

void Merger::mergeTile(const NodeInfo &nodeInfo, const TileId &tileId
                       , const merge::Input::list &parentSource)
{
    if (!world_.exists(tileId)) {
        // no data below
        return;
    }

    vadstena::storage::TIDGuard tg(str(boost::format("%s") % tileId), true);

    bool g(generate_.exists(tileId));
    bool atBottom(tileId.lod >= generate_.maxLod());

    LOG(info2) << "(glue) Processing tile " << tileId << ".";

    // terminate descent if at bottom and nothing is going to be generated
    if (atBottom && !g) { return; }

    // process tile
    auto tile(processTile(nodeInfo, tileId, parentSource
                          , Constraints(*this, g)));

    if (tile) {
        self_.setTile(tileId, tile.getMesh(), tile.getAtlas()
                      , tile.getNavtile(), &nodeInfo);
    } else if (g && !tile.source.empty()) {
        auto topSourceId(tile.source.back().id());
        if (topSourceId != topId_) {
            // tile references single existing tile in other set -> store
            LOG(info4) << "Setting reference " << topSourceId;
            self_.addReference(tileId, topSourceId);
        }
    }

    if (g) {
        (++progress_).report(utility::Progress::ratio_t(5, 1000), "(glue) ");
    }

    // do not descent if we are at the bottom
    if (atBottom) { return; }

    tg.pop();

    // OK, process children
    for (const auto &child : children(tileId)) {
        mergeTile(nodeInfo.child(child), child, tile.source);
    }
}

merge::Output Merger::processTile(const NodeInfo &nodeInfo
                                  , const TileId &tileId
                                  , const merge::Input::list &parentSource
                                  , const Constraints &constraints)
{
    merge::Input::list input;
    {
        merge::Input::Id id(0);
        for (const auto *ts : src_) {
            merge::Input t(id++, *ts, tileId, nodeInfo);
            if (t) { input.push_back(t); }
        }
    }
    return merge::mergeTile(tileId, nodeInfo, input, parentSource
                            , constraints);
}

} // namespace

void TileSet::createGlue(const const_ptrlist &sets)
{
    if (sets.size() < 2) {
        LOG(info3) << "(glue) Too few sets to glue together ("
                   << sets.size() << ").";
        return;
    }

    LOG(info3) << "(glue) Calculating generate set.";

    const auto *dumpRoot(getDumpDir());
    const auto &referenceFrame(getProperties().referenceFrame);

    // lod range of all sets
    const LodRange lr(range(sets));
    LOG(info2) << "LOD range: " << lr;

    // get all indices
    auto all(tileIndices(sets, lr));
    auto &top(all.back());
    dumpTileIndex(dumpRoot, "top", top, referenceFrame);

    top.round();
    const auto topUp(TileIndex(top).complete());
    const auto topDown(TileIndex(top).completeDown());

    dumpTileIndex(dumpRoot, "top-round", top, referenceFrame);
    dumpTileIndex(dumpRoot, "top-up", topUp, referenceFrame);
    dumpTileIndex(dumpRoot, "top-down", topDown, referenceFrame);

    boost::sub_range<TileIndexList> rest(all.begin(), std::prev(all.end()));

    // output generate set
    boost::optional<TileIndex> generate;

    for (const auto &r : rest) {
        auto rUp(TileIndex(r).complete());
        auto rDown(TileIndex(r).completeDown());

        auto i1(topUp.intersect(rDown));
        auto i2(topDown.intersect(rUp));

        if (!generate) {
            generate = unite(i1, i2);
        } else {
            generate = generate->intersect(unite(i1, i2));
        }
    }

    dumpTileIndex(dumpRoot, "generate", *generate, referenceFrame);
    LOG(info1) << "generate: " << generate->count();

    if (generate->empty()) {
        LOG(warn3) << "(glue) Nothing to generate. Bailing out.";
        return;
    }

    LOG(info3) << "(glue) Generate set calculated.";

    // create merge
    Merger merger(detail(), *generate, sets);

    // copy position
    setPosition(sets.back()->getProperties().position);

    // done
}

} } // namespace vadstena::vts
