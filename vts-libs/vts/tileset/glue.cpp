#include <boost/format.hpp>

#include "utility/progress.hpp"

#include "../io.hpp"

#include "./detail.hpp"
#include "./merge.hpp"

namespace fs = boost::filesystem;

namespace vadstena { namespace vts {

namespace {

inline std::vector<TileIndex> tileIndices(const TileSet::const_ptrlist &sets
                                          , const LodRange &lodRange)
{
    std::vector<TileIndex> indices;
    for (const auto ts : sets) {
        indices.push_back
            (ts->tileIndex(lodRange).simplify(TileIndex::Flag::mesh));
    }
    return indices;
}

inline TileIndices tileIndices(const std::vector<TileIndex> &tis
                               , std::size_t size
                               = std::numeric_limits<std::size_t>::max())
{
    size = std::min(size, tis.size());

    TileIndices indices;
    for (const auto &ti : tis) {
        indices.push_back(&ti);
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
                   , const TileIndex &index)
{
    if (!root) { return; }
    LOG(info1) << "Dumping tileindex " << name << ".";
    dumpAsImages(root / name, index);
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

struct Merger {
public:
    Merger(TileSet::Detail &self, const TileIndex &generate
           , const TileSet::const_ptrlist &src)
        : self(self), world(generate), generate(generate)
        , src(src), progress(generate.count())
    {
        // make world complete
        world.complete();
    }

    void operator()() {
        mergeTile(NodeInfo(self.referenceFrame));
    }


private:
    /** Merge subtree starting at index.
     *  Calls itself recursively.
     */
    void mergeTile(const NodeInfo &nodeInfo
                   , const TileId &tileId = TileId()
                   , const merge::Input::list &parentSource
                   = merge::Input::list()
                   , int quadrant = -1
                   , bool parentGenerated = false);

    /** Generates new tile as a merge of tiles from other tilesets.
     */
    merge::Output generateTile(const NodeInfo &nodeInfo
                               , const TileId &tileId
                               , const merge::Input::list &parentSource
                               , int quadrant);


    TileSet::Detail &self;
    TileIndex world;
    const TileIndex &generate;
    const TileSet::const_ptrlist &src;

    utility::Progress progress;
};

void Merger::mergeTile(const NodeInfo &nodeInfo, const TileId &tileId
                       , const merge::Input::list &parentSource
                       , int quadrant, bool parentGenerated)
{
    if (!world.exists(tileId)) {
        // no data below
        return;
    }

    merge::Input::list source;

    auto g(generate.exists(tileId));
    if (g) {
        LOG(info2) << "(glue) Processing tile " << tileId << ".";

        bool thisGenerated(false);

        if (!parentGenerated) {
            // if (auto t = self.getTile(self.parent(tileId), std::nothrow)) {
            //     // no parent was generated and we have sucessfully loaded parent
            //     // tile from existing content as a fallback tile!

            //     // TODO: since this is merge result its information can be
            //     // outdated
            //     MergeInput::list loadedParentTiles;
            //     loadedParentTiles.push_back
            //         (MergeInput(t.get(), nullptr, tileId));

            //     quadrant = child(tileId);
            //     tile = generateTile(tileId, loadedParentTiles
            //                         , incidentTiles, quadrant);
            //     thisGenerated = true;
            // }
        }

        if (!thisGenerated) {
            // regular generation: generate tile and remember its sources (used
            // in children generation)
            auto tile(generateTile(nodeInfo, tileId, parentSource, quadrant));
            source = tile.source;

            if (tile) {
                self.setTile(tileId, tile.getMesh(), tile.getAtlas()
                             , tile.getNavtile());
            }
        }

        (++progress).report(utility::Progress::ratio_t(5, 1000), "(glue) ");
    }

    // can we go down?
    if (tileId.lod >= generate.maxLod()) {
        // no way down
        return;
    }

    // OK, process children
    quadrant = 0;
    for (const auto &child : children(tileId)) {
        mergeTile(nodeInfo.child(child), child, source, quadrant++, g);
    }
}

merge::Output Merger::generateTile(const NodeInfo &nodeInfo
                                   , const TileId &tileId
                                   , const merge::Input::list &parentSource
                                   , int quadrant)
{
    LOG(info3) << "Generate tile: " << tileId;

    // create input
    merge::Input::list input;
    {
        merge::Input::Id id(0);
        for (const auto &ts : src) {
            merge::Input t(id++, self.other(*ts), tileId, nodeInfo);
            if (t) { input.push_back(t); }
        }
    }

    auto tile(merge::mergeTile(tileId, input, parentSource, quadrant));

    // TODO: analyze tile and store if it is proper glue tile

    // done
    return tile;
}

} // namespace

void TileSet::createGlue(const const_ptrlist &sets)
{
    LOG(info3) << "(glue) Calculating generate set.";

    const auto *dumpRoot(getDumpDir());

    // lod range of the world
    auto lodRange(range(sets));

    // clone simplified indices
    auto indices(tileIndices(sets, lodRange));
    dump(dumpRoot, "indices", indices);

    auto tsUpdate(indices.back());
    if (tsUpdate.empty()) {
        LOG(warn3) << "(glue) Nothing to generate. Bailing out.";
        return;
    }

    dumpTileIndex(dumpRoot, "tsUpdate", tsUpdate);
    tsUpdate.growDown();
    dumpTileIndex(dumpRoot, "tsUpdate-gd", tsUpdate);

    auto tsPost(unite(tileIndices(indices)));
    dumpTileIndex(dumpRoot, "tsPost", tsPost);
    tsPost.growUp();
    dumpTileIndex(dumpRoot, "tsPost-gu", tsPost);

    auto tsPre(unite(tileIndices(indices, indices.size() - 1)));
    dumpTileIndex(dumpRoot, "tsPre", tsPre);
    tsPre.growUp();
    dumpTileIndex(dumpRoot, "tsPre-gu", tsPre);
    tsPre.invert();
    dumpTileIndex(dumpRoot, "tsPre-gu-inv", tsPre);

    auto generate(tsPost.intersect(unite(tsUpdate, tsPre)));
    dumpTileIndex(dumpRoot, "generate", generate);

    LOG(info1) << "generate: " << generate.count();

    if (generate.empty()) {
        LOG(warn3) << "(glue) Nothing to generate. Bailing out.";
        return;
    }

    Merger(detail(), generate, sets)();
}

} } // namespace vadstena::vts
