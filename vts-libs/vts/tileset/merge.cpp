#include <boost/format.hpp>

#include "./detail.hpp"

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
        LOG(warn3) << "(merge-in) Nothing to merge in. Bailing out.";
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

    LOG(info4) << "generate: " << generate.count();
}

} } // namespace vadstena::vts
