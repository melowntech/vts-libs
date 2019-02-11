/**
 * Copyright (c) 2017 Melown Technologies SE
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * *  Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#include <iterator>

#include <boost/format.hpp>
#include <boost/range/sub_range.hpp>

#include "utility/progress.hpp"
#include "utility/path.hpp"

#include "../io.hpp"

#include "../../storage/tidguard.hpp"
#include "../meshop.hpp"
#include "detail.hpp"
#include "merge.hpp"

namespace fs = boost::filesystem;

namespace vtslibs { namespace vts {

namespace {

typedef std::vector<TileIndex> TileIndexList;

inline TileIndexList
tileIndices(const TileSet::list &sets, const LodRange &lodRange
            , TileIndex::Flag::value_type mask)
{
    std::vector<TileIndex> indices;
    for (const auto &set : sets) {
        indices.push_back(set.tileIndex(lodRange).simplify(mask));

        // and clamp them to desired range
        indices.back().clamp(lodRange);
    }
    return indices;
}

LodRange range(const TileSet::list &sets
               , const TileIndex::Flag::value_type mask = TileIndex::Flag::any)
{
    // no set -> no lod range
    if (sets.empty()) { return LodRange::emptyRange(); }

    // start with LOD 0
    LodRange r(0, 0);

    for (const auto &set : sets) {
        if (mask != TileIndex::Flag::any) {
            // NB: probably can be done more efficiently not using statMask
            auto stat(set.tileIndex().statMask(mask, mask));
            r = unite(r, stat.lodRange);
        } else {
            r = unite(r, set.lodRange());
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
                   , const TileIndex &index)
{
    if (!root) { return; }
    LOG(info1) << "Dumping tileindex " << name << ".";
    auto filename(root / utility::addExtension(name, ".index"));
    create_directories(filename.parent_path());
    index.save(filename);
}

inline void dump(const char *root, const boost::filesystem::path &dir
                 , const std::vector<TileIndex> &tileIndices)
{
    if (!root) { return; }

    int i(0);
    for (const auto &ti : tileIndices) {
        dumpTileIndex(root, dir / str(boost::format("%03d") % i), ti);
        ++i;
    }
}

MeshOpInput::DataSource::list sources(const TileSet::list &sets)
{
    MeshOpInput::DataSource::list out;
    for (const auto &set : sets) {
        out.push_back(tilesetDataSource(set.detail()));
    }
    return out;
}

struct Merger {
public:
    Merger(TileSet::Detail &glue
           , const TileIndex &generate
           , const TileIndex &navtileGenerate
           , const TileSet::list &srcSets
           , const GlueCreationOptions &options)
        : glue_(glue), world_(generate), generate_(generate)
        , navtileGenerate_(navtileGenerate)
        , src_(sources(srcSets)), top_(srcSets.back().detail())
        , topId_(src_.size() - 1), progress_(generate_.count())
        , options_(options)
    {
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
    const MeshOpInput::DataSource::list src_;
    const TileSet::Detail &top_;
    const merge::Input::Id topId_;

    utility::Progress progress_;

    const GlueCreationOptions options_;
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
    vtslibs::storage::TIDGuard tg(str(boost::format("%s") % tileId), true);

    const bool g(generate_.exists(tileId));
    const bool atBottom(tileId.lod >= generate_.maxLod());

    auto descend([&](const merge::TileSource &source)
    {
        if (g) {
            (++progress_).report(utility::Progress::ratio_t(5, 1000)
                                 , "(glue) ");
            if (options_.progress) { options_.progress->tile(); }
        }

        // do not descent if we are at the bottom
        if (atBottom) { return; }

        // remove this tile from thread name
        tg.pop();

        // this tile is processed, go after children
        for (const auto &child : children(tileId)) {
            mergeTile(nodeInfo.child(child), child, source);
        }
    });

    LOG(info2) << "(glue) Processing tile " << tileId
               << " (" << (g ? 'G': 'g')
               << (atBottom ? 'B': 'b') << ").";

    // terminate descent if at bottom and nothing is going to be generated
    if (atBottom && !g) {
        LOG(info1) << "Bottom and nothing to generate there.";
        return;
    }

    if (!nodeInfo.productive()) {
        // unproductive node, immediately descend
        // forward parent source
        descend(parentSource);
        return;
    }

    const bool ng(navtileGenerate_.exists(tileId));

    // process tile
    auto tile(processTile(nodeInfo, tileId, parentSource
                              , Constraints(*this, g, ng)));

    if (tile) {
        // place tile to glue
        glue_.setTile(tileId
                      , tile.tile(options_.textureQuality)
                      .setAlien(isAlienTile(tile))
                      , &nodeInfo);
    }

    // this tile is processed, go after children
    descend(tile.source);
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
        for (const auto &source : src_) {
            merge::Input t(id++, source, tileId, &nodeInfo);
            if (t) { input.push_back(t); }
        }
    }

    // run merge operation
    return merge::mergeTile(tileId, nodeInfo, input, parentSource
                            , constraints, options_);
}

// Make black-white tileindex complete to given ceiling and round -> each
// node has parent/is accessible and quad condition is satisfied for
// all tiles
// NB: idempotent operation: f(f(x)) = f(x)
void finalize(TileIndex &index, Lod ceiling)
{
    index.complete(1, ceiling);
    index.round();
}

// Expects sane tilesets: tileset is compact representation of its finest LOD
// data.  The representation need not to be perfect (additonal tiles at lower
// LODS) but these imperfections are not meant to significantly influence merge
// output or propagate to finer LODs NB: union and intersection of SoI is again
// a SoI
TileIndex sphereOfInfluenceForMerge(const TileIndex &i, Lod ceiling)
{
    auto index(i);
    index.completeDownFromBottom();
    finalize(index, ceiling);
    return index;
}

TileIndex buildGenerateSet(const char *dumpRoot
                           , const LodRange &lr
                           , const TileSet::list &sets
                           , TileIndex::Flag::value_type mask
                           , Lod &glueCeiling)
{
    // get all indices from all sets in full LOD range
    auto all(tileIndices(sets, lr, mask));

    // topmost set's tile index
    const auto &top(all.back());

    // rest of tilesets (without top-level set)
    boost::sub_range<TileIndexList> rest(all.begin(), std::prev(all.end()));

    // calculate glue ceiling lod:
    // 1) take maximum of maximum lods from rest sets
    const auto bottomMostIndex(std::max_element
                               (rest.begin(), rest.end()
                                , [](const TileIndex &l, const TileIndex &r) {
                                   return (l.maxLod() > r.maxLod());
                               }));

    // 2) take minimum of bottom most rest lod and top's minimum
    glueCeiling = std::min(Lod(bottomMostIndex->maxLod() + 1), top.minLod());

    // create spheres of influence of all indices and intersect them
    auto generate(sphereOfInfluenceForMerge(top, glueCeiling));

    dumpTileIndex(dumpRoot, "top-sphere", generate);


    for (const auto &r : rest) {
        // intersect generate set with other spheres of influence
        auto rsoi(sphereOfInfluenceForMerge(r, glueCeiling));
        dumpTileIndex(dumpRoot, "rest-sphere", rsoi);
        generate = generate.intersect(rsoi);
    }

    return generate;
}

TileIndex optimizeGenerateSet(const TileIndex & generateSet
                              , const char *dumpRoot
                              , const LodRange &lr
                              , const TileSet::list &sets
                              , Lod glueCeiling)
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

    dumpTileIndex(dumpRoot, "watertight-union", watertight);

    // invert and intersect with generate set
    watertight.invert(1);

    dumpTileIndex(dumpRoot, "watertight-inverse", watertight);

    watertight = watertight.intersect(generateSet);

    dumpTileIndex(dumpRoot, "optimized-raw", watertight);

    finalize(watertight, glueCeiling);

    return watertight;
}

TileIndex buildGenerateSet(const TileSet::list &sets, const LodRange &lr
                          , const GlueCreationOptions::GenerateSetManipulator
                              &generateSetManipulator
                                = GlueCreationOptions::GenerateSetManipulator())
{
    LOG(info3) << "(glue) Calculating generate set.";

    const auto *dumpRoot(getDumpDir());

    Lod glueCeiling(0);

    /* For tilesets with LOD-uneven bottom, the simple growDownFromBottom
    does not work. For this purpose, the tiles that would be in the tileset if
    it was generated to have flat bottom at the fines LOD are marked as
    'influenced' in the tileindex - they do not physically exist but they should
    be generated from tileset's data (they are influenced by some meshes above
    them) if needed.
    Same behavior could be achieved if the tileset was split at the common
    bottom into two (or more) tilesets that would enter merge individually.
    */
    auto generateRaw(buildGenerateSet(dumpRoot, lr, sets
                        , TileIndex::Flag::mesh | TileIndex::Flag::influenced
                        , glueCeiling));

    dumpTileIndex(dumpRoot, "generate-raw", generateRaw);

    auto optimized(optimizeGenerateSet( generateRaw, dumpRoot
                                      , lr, sets, glueCeiling));

    if (generateSetManipulator) {
        generateSetManipulator(optimized);
    }

    return optimized;
}


} // namespace

TileSet::GlueStatistics
TileSet::analyzeGlue(const list &sets, const GlueCreationOptions&)
{
    GlueStatistics stat;

    if (sets.size() >= 2) {
        // LOD range of all sets
        const LodRange lr(range(sets));
        LOG(info2) << "LOD range: " << lr;
        auto generate(buildGenerateSet(sets, lr));
        stat.tilesToGenerate = generate.count();
    }

    return stat;
}

void TileSet::createGlue(TileSet &glue, const list &sets
                         , const GlueCreationOptions &options)
{
    if (sets.size() < 2) {
        LOG(info3) << "(glue) Too few sets to glue together ("
                   << sets.size() << ").";
        return;
    }

    LOG(info3) << "(glue) Calculating generate set.";

    const auto *dumpRoot(getDumpDir());

    // LOD range of all sets
    const LodRange lr(range(sets));
    LOG(info2) << "LOD range: " << lr;

    Lod glueCeiling(0);

    auto generate(buildGenerateSet(sets, lr, options.generateSetManipulator));

    dumpTileIndex(dumpRoot, "generate", generate);
    LOG(info1) << "generate: " << generate.count();

    if (generate.empty()) {
        LOG(warn3) << "(glue) Nothing to generate. Bailing out.";
        return;
    }

    LOG(info3) << "(glue) Generate set calculated.";

    // calculate navtile generate set
    const LodRange navLr( range(sets, TileIndex::Flag::navtile) );

    LOG(info2) << "Navtile LOD range: " << navLr;

    // regarding influenced tiles look at the remark in buildGenerateSet for
    // normal melh tiles
    auto navtileGenerateRaw
        (buildGenerateSet(dumpRoot, navLr, sets
                , TileIndex::Flag::navtile | TileIndex::Flag::influenced
                , glueCeiling));

    dumpTileIndex(dumpRoot, "generate-navtile-raw", navtileGenerateRaw);

    auto navtileGenerate(optimizeGenerateSet(navtileGenerateRaw, dumpRoot
                                             , navLr, sets, glueCeiling));
    dumpTileIndex(dumpRoot, "generate-navtile", navtileGenerate);

    // run merge
    Merger(glue.detail()
           , generate, navtileGenerate, sets, options);

    // copy position from top dataset
    glue.setPosition(sets.back().getProperties().position);

    // done
}

} } // namespace vtslibs::vts
