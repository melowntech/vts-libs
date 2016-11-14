#include <iterator>

#include <boost/format.hpp>
#include <boost/range/sub_range.hpp>

#include "utility/progress.hpp"
#include "utility/path.hpp"

#include "../io.hpp"

#include "../../storage/tidguard.hpp"
#include "../meshop.hpp"
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

LodRange range( const TileSet::const_ptrlist &sets
              , const TileIndex::Flag::value_type mask = TileIndex::Flag::any)
{
    // no set -> no lod range
    if (sets.empty()) { return LodRange::emptyRange(); }

    // start with LOD 0
    LodRange r(0, 0);

    for (const auto &set : sets) {
        if (mask != TileIndex::Flag::any) {
            // NB: probably can be done more efficiently not using statMask
            auto stat(set->tileIndex().statMask(mask, mask));
            r = unite(r, stat.lodRange);
        } else {
            r = unite(r, set->lodRange());
        }
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
    Merger(TileSet::Detail &glue
           , const TileIndex &generate
           , const TileIndex &navtileGenerate
           , const TileSet::const_ptrlist &srcSets
           , const GlueCreationOptions &options)
        : glue_(glue), world_(generate), generate_(generate)
        , navtileGenerate_(navtileGenerate)
        , src_(details(glue, srcSets)), top_(*src_.back())
        , topId_(src_.size() - 1), progress_(generate_.count())
        , options_(options)
    {
        // update merge options
        mergeOptions_.clip = options.clip;

        // make world complete
        world_.complete();

        // run
        mergeTile(NodeInfo(glue_.referenceFrame));
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

    bool isAlienTile(const merge::Output &tile) const;
    bool isGlueTile(const merge::Output &tile) const;

    TileSet::Detail &glue_;
    TileIndex world_;
    const TileIndex &generate_;
    const TileIndex &navtileGenerate_;
    const DetailList src_;
    const TileSet::Detail &top_;
    const merge::Input::Id topId_;

    utility::Progress progress_;

    const GlueCreationOptions options_;
    merge::MergeOptions mergeOptions_;
};

inline bool Merger::isAlienTile(const merge::Output &tile) const
{
    auto size(tile.source.mesh.size());
    auto srcSize(src_.size());

    if ((size + 1) != srcSize) { return false; }

    // possibly generated from sets other than top set
    if (tile.source.mesh.back().id() == topId_) {
        // contains top set -> cannot be a glue
        return false;
    }

    // tile must be fully derived from other sets (such tile cannot exist in
    // other glues)
    return tile.fullyDerived();
}

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

    if (isAlienTile(tile)) { return true; }

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
        // place alien tiles to alien glue and proper tiles to glue
        glue_.setTile(tileId
                      , tile.tile(options_.textureQuality)
                      .setAlien(!options_.generateReferences
                                && isAlienTile(tile))
                      , &nodeInfo);

    } else if (g && options_.generateReferences
               && !tile.source.mesh.empty())
    {
        // we are allowed to generate references
        // no tile generated but there are some data to generate it
        auto topSourceId(tile.source.mesh.back().id());
        if (topSourceId != topId_) {
            // tile references single tile in other set -> store reference
            LOG(info1) << "Setting reference " << topSourceId + 1;
            glue_.setReferenceTile(tileId, topSourceId + 1, &nodeInfo);
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
                            , constraints, mergeOptions_);
}

// Make black-white tileindex complete to ceiling and round -> each 
// node has parent/is accessible and quad condition is satisfied for
// all tiles
// NB: idempotent operation: f(f(x)) = f(x)
void finalize(TileIndex & index) {
    index.complete(1,true);
    index.round();
}

// Expects sane tilesets: tileset is compact representation of its finest LOD data.
// The representation need not to be perfect (additonal tiles at lower LODS) but
// these imperfections are not meant to significantly influence merge output or
// propagate to finer LODs
// NB: union and intersection of SoI is again a SoI
TileIndex sphereOfInfluenceForMerge(const TileIndex & i) {
    auto index(i);
    index.completeDownFromBottom();
    finalize(index);
    return index;
}

TileIndex buildGenerateSet(const char *dumpRoot
                           , const std::string &referenceFrameId
                           , const LodRange &lr
                           , const TileSet::const_ptrlist &sets
                           , TileIndex::Flag::value_type mask)
{
    (void) dumpRoot;
    (void) &referenceFrameId;

    // get all indices from all sets in full LOD range
    auto all(tileIndices(sets, lr, mask));

    // create spheres of influence of all indices and intersect them
    auto generate( sphereOfInfluenceForMerge(all.back()) );

    dumpTileIndex(dumpRoot, "top-sphere", generate, referenceFrameId);

    boost::sub_range<TileIndexList> rest(all.begin(), std::prev(all.end()));

    for (const auto &r : rest) { 
        // intersect generate set with other spheres of influence
        dumpTileIndex(dumpRoot, "rest-sphere", sphereOfInfluenceForMerge(r), referenceFrameId);
        generate = generate.intersect( sphereOfInfluenceForMerge(r) );
    }

    return generate;
}

TileIndex optimizeGenerateSet(const TileIndex & generateSet
                           , const char *dumpRoot
                           , const std::string &referenceFrameId
                           , const LodRange &lr
                           , const TileSet::const_ptrlist &sets)
{
    // get all indices from all sets in full LOD range
    auto all(tileIndices(sets, lr, TileIndex::Flag::watertight));

    // bottom dataset is discarded, top handled separatelly
    boost::sub_range<TileIndexList> subset( std::next(all.begin())
                                          , std::prev(all.end()));

    // unite watertight tileindices - that is where the glue will not 
    // be generated
    TileIndex watertight;

    for (const auto &r : subset) {
        if (watertight.empty()) {
            watertight = TileIndex(r).completeDownFromBottom();
        } else {
            // FIX: fix case when last LOD has only non-watertight tiles - in
            // this case completeDownFromBottom won't work as intended
            watertight = unite(watertight, TileIndex(r).completeDownFromBottom());
        }
    }

    // add watertight from top dataset - do not complete down as this may be
    // needed to override lower LODS 
    watertight = unite(watertight, all.back()); 

    // now we have tileset showing where at least one (except bottom) set is
    // watertight -> no glue

    dumpTileIndex(dumpRoot, "watertight-union", watertight, referenceFrameId);

    // invert and intersect with generate set
    watertight.invert(1);

    dumpTileIndex(dumpRoot, "watertight-inverse", watertight, referenceFrameId);

    watertight = watertight.intersect(generateSet);

    dumpTileIndex(dumpRoot, "optimized-raw", watertight, referenceFrameId);

    finalize(watertight);
    
    return watertight;
}


} // namespace

void TileSet::createGlue(TileSet &glue, const const_ptrlist &sets
                         , const GlueCreationOptions &options)
{
    if (sets.size() < 2) {
        LOG(info3) << "(glue) Too few sets to glue together ("
                   << sets.size() << ").";
        return;
    }

    LOG(info3) << "(glue) Calculating generate set.";

    const auto *dumpRoot(getDumpDir());
    const auto &referenceFrameId(glue.getProperties().referenceFrame);

    // LOD range of all sets
    const LodRange lr(range(sets));
    LOG(info2) << "LOD range: " << lr;

    auto generateRaw(buildGenerateSet(dumpRoot, referenceFrameId, lr, sets
                                   , TileIndex::Flag::mesh));

    dumpTileIndex(dumpRoot, "generate-raw", generateRaw, referenceFrameId);

    auto generate(optimizeGenerateSet( generateRaw, dumpRoot
                                     , referenceFrameId, lr, sets));

    dumpTileIndex(dumpRoot, "generate", generate, referenceFrameId);
    LOG(info1) << "generate: " << generate.count();

    if (generate.empty()) {
        LOG(warn3) << "(glue) Nothing to generate. Bailing out.";
        return;
    }

    LOG(info3) << "(glue) Generate set calculated.";

    generate = TileIndex();

    // calculate navtile generate set
    const LodRange navLr( range(sets, TileIndex::Flag::navtile) );

    LOG(info2) << "Navtile LOD range: " << navLr;
    
    auto navtileGenerate
        (buildGenerateSet(dumpRoot, referenceFrameId, navLr, sets
                          , TileIndex::Flag::navtile));
    dumpTileIndex(dumpRoot, "generate-navtile", navtileGenerate
                  , referenceFrameId);

    // run merge
    Merger(glue.detail()
           , generate, navtileGenerate, sets, options);

    // copy position from top dataset
    glue.setPosition(sets.back()->getProperties().position);

    // done
}

} } // namespace vadstena::vts
