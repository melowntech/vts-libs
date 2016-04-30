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
            , TileIndex::Flag::value_type mask)
{
    std::vector<TileIndex> indices;
    for (const auto &set : sets) {
        indices.push_back(set->tileIndex(lodRange).simplify(mask));
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
                   , const std::string &referenceFrameId)
{
    if (!root) { return; }
    LOG(info1) << "Dumping tileindex " << name << ".";
    auto filename(root / utility::addExtension(name, ".index"));
    create_directories(filename.parent_path());
    index.save(filename);

    auto rfFilename(root / utility::addExtension(name, ".rframe"));
    utility::write(rfFilename, referenceFrameId.data()
                   , referenceFrameId.size());
}

inline void dump(const char *root, const boost::filesystem::path &dir
                 , const std::vector<TileIndex> &tileIndices
                 , const std::string &referenceFrameId)
{
    if (!root) { return; }

    int i(0);
    for (const auto &ti : tileIndices) {
        dumpTileIndex(root, dir / str(boost::format("%03d") % i), ti
                      , referenceFrameId);
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
           , const TileIndex &navtileGenerate
           , const TileSet::const_ptrlist &srcSets)
        : self_(self), world_(generate), generate_(generate)
        , navtileGenerate_(navtileGenerate)
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
        Constraints(const Merger &merger, bool generable
                    , bool generateNavtile)
            : merge::MergeConstraints(generable, generateNavtile)
            , merger(merger)
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
                   , const merge::TileSource &parentSource
                   = merge::TileSource());

    /** Generates new tile as a merge of tiles from other tilesets.
     */
    merge::Output processTile(const NodeInfo &nodeInfo
                              , const TileId &tileId
                              , const merge::TileSource &parentSource
                              , const Constraints &constraints);

    bool isGlueTile(const merge::Output &tile) const;

    TileSet::Detail &self_;
    TileIndex world_;
    const TileIndex &generate_;
    const TileIndex &navtileGenerate_;
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

    auto size(tile.source.mesh.size());
    auto srcSize(src_.size());

    // sanity check
    if (!size) { return false; }

    // special case
    if (tile.source.mesh.front().id() == topId_) {
        // generated only from top set, must be derived tile to be glue tile
        return tile.derived(0);
    }

    // if tile is generated from all input sets -> glue
    if (size == srcSize) {
        return true;
    }

    if ((size + 1) == srcSize) {
        // possibly generated from sets other than top set
        if (tile.source.mesh.back().id() == topId_) {
            // contains top set -> cannot be a glue
            return false;
        }

        // tile must be fully derived from other sets (such tile cannot exist in
        // other glues)
        return tile.fullyDerived();
    }

    // anything else -> not a glue
    return false;
}

void Merger::mergeTile(const NodeInfo &nodeInfo, const TileId &tileId
                       , const merge::TileSource &parentSource)
{
    if (!nodeInfo.valid() || !world_.exists(tileId)) {
        // no data here and below
        return;
    }

    // thread name contains tile
    vadstena::storage::TIDGuard tg(str(boost::format("%s") % tileId), true);

    const bool g(generate_.exists(tileId));
    const bool atBottom(tileId.lod >= generate_.maxLod());

    LOG(info2) << "(glue) Processing tile " << tileId
               << " (" << (g ? 'G': 'g')
               << (atBottom ? 'B': 'b') << ").";

    // terminate descent if at bottom and nothing is going to be generated
    if (atBottom && !g) {
        LOG(info1) << "Bottom and nothing to generate there.";
        return;
    }

    const bool ng(navtileGenerate_.exists(tileId));

    // process tile
    auto tile(processTile(nodeInfo, tileId, parentSource
                          , Constraints(*this, g, ng)));

    if (tile) {
        // tile generated, store into glue
        self_.setTile(tileId, tile.tile(), &nodeInfo);
    } else if (g && !tile.source.mesh.empty()) {
        // no tile generated but there are some data to generate it
        auto topSourceId(tile.source.mesh.back().id());
        if (topSourceId != topId_) {
            // tile references single tile in other set -> store reference
            LOG(info1) << "Setting reference " << topSourceId + 1;
            self_.setReferenceTile(tileId, topSourceId + 1, &nodeInfo);
        }
    }

    if (g) {
        (++progress_).report(utility::Progress::ratio_t(5, 1000), "(glue) ");
    }

    // do not descent if we are at the bottom
    if (atBottom) { return; }

    // remove this tile from thread name
    tg.pop();

    // this tile is processed, go after children
    for (const auto &child : children(tileId)) {
        mergeTile(nodeInfo.child(child), child, tile.source);
    }
}

merge::Output Merger::processTile(const NodeInfo &nodeInfo
                                  , const TileId &tileId
                                  , const merge::TileSource &parentSource
                                  , const Constraints &constraints)
{
    // fetch input
    merge::Input::list input;
    {
        merge::Input::Id id(0);
        for (const auto *ts : src_) {
            merge::Input t(id++, *ts, tileId, &nodeInfo);
            if (t) { input.push_back(t); }
        }
    }

    // run merge operation
    return merge::mergeTile(tileId, nodeInfo, input, parentSource
                            , constraints);
}

TileIndex buildGenerateSet(const char *dumpRoot
                           , const std::string &referenceFrameId
                           , const LodRange &lr
                           , const TileSet::const_ptrlist &sets
                           , TileIndex::Flag::value_type mask)
{
    // get all indices from all sets in full LOD range
    auto all(tileIndices(sets, lr, mask));
    auto &top(all.back());
    dumpTileIndex(dumpRoot, "top", top, referenceFrameId);

    // make top round (i.e. quad condition is satisfied for every tile)
    top.round();
    // make complete up and down trees for top (up: every tile has parent, down:
    // every tile has all children)
    const auto topUp(TileIndex(top).growUp());
    const auto topDown(TileIndex(top).completeDown());

    dumpTileIndex(dumpRoot, "top-round", top, referenceFrameId);
    dumpTileIndex(dumpRoot, "top-up", topUp, referenceFrameId);
    dumpTileIndex(dumpRoot, "top-down", topDown, referenceFrameId);

    // grab all tile indices except the top one
    boost::sub_range<TileIndexList> rest(all.begin(), std::prev(all.end()));

    // output generate set
    boost::optional<TileIndex> generate;

    for (const auto &r : rest) {
        // make up and down complete tree
        auto rUp(TileIndex(r).complete());
        auto rDown(TileIndex(r).completeDown());

        // intersect up and down trees with top up and down trees
        auto i1(topUp.intersect(rDown));
        auto i2(topDown.intersect(rUp));

        if (!generate) {
            // first iteration -> just unite both trees
            generate = unite(i1, i2);
        } else {
            // next iteration -> unite up, down and intersect with the result
            generate = generate->intersect(unite(i1, i2));
        }
    }

    // done
    return *generate;
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
    const auto &referenceFrameId(getProperties().referenceFrame);

    // LOD range of all sets
    const LodRange lr(range(sets));
    LOG(info2) << "LOD range: " << lr;

    auto generate(buildGenerateSet(dumpRoot, referenceFrameId, lr, sets
                                   , TileIndex::Flag::mesh));

    dumpTileIndex(dumpRoot, "generate", generate, referenceFrameId);
    LOG(info1) << "generate: " << generate.count();

    if (generate.empty()) {
        LOG(warn3) << "(glue) Nothing to generate. Bailing out.";
        return;
    }

    LOG(info3) << "(glue) Generate set calculated.";

    // calculate navtile generate set
    auto navtileGenerate
        (buildGenerateSet(dumpRoot, referenceFrameId, lr, sets
                          , TileIndex::Flag::navtile));
    dumpTileIndex(dumpRoot, "generate-navtile", navtileGenerate
                  , referenceFrameId);

    // run merge
    Merger(detail(), generate, navtileGenerate, sets);

    // copy position from top dataset
    setPosition(sets.back()->getProperties().position);

    // done
}

} } // namespace vadstena::vts
