#include "dbglog/dbglog.hpp"
#include "utility/binaryio.hpp"

#include "./tileindex.hpp"
#include "./tileop.hpp"

namespace vadstena { namespace tilestorage {

namespace fs = boost::filesystem;

namespace {
    const char TILE_INDEX_IO_MAGIC[7] = { 'T', 'I', 'L', 'E', 'I', 'D', 'X' };
}

namespace {

/** Converts coordinate to grid defined by reference point and cell size.
 *  Coordinates not on grid are rounded up.
 */
long gridUp(long value, long origin, unsigned long size)
{
    auto oft(value - origin);
    if (oft < 0) {
        return origin + (oft / size);
    }

    return origin + ((size - 1 + oft) / size);
}

/** Converts coordinate to grid defined by reference point and cell size.
 *  Coordinates not on grid are rounded down.
 */
long gridDown(long value, long origin, unsigned long size)
{
    auto oft(value - origin);
    if (oft < 0) {
        return origin + ((size - 1 - oft) / size);
    }

    return origin + (oft / size);
}

Extents grid(const Extents &extents, const Alignment &alignment
             , unsigned long size)
{
    return {
        gridDown(extents.ll(0), alignment(0), size)
        , gridDown(extents.ll(1), alignment(1), size)
        , gridUp(extents.ur(0), alignment(0), size)
        , gridUp(extents.ur(1), alignment(1), size)
    };
}

} // namespace

TileIndex::TileIndex(const Alignment &alignment, long baseTileSize
                     , Extents extents, LodRange lodRange
                     , const TileIndex *other)
    : baseTileSize_(baseTileSize)
    , minLod_(lodRange.min)
{
    // include old definition if non-empty
    if (other && !other->empty()) {
        // something present in on-disk data
        extents = unite(extents, other->extents());
        lodRange = unite(lodRange, other->lodRange());
    }

    // maximal tile size (at lowest lod)
    auto ts(tileSize(baseTileSize, lodRange.min));

    // calculate proper extents at lowest lod
    auto ex(grid(extents, alignment, ts));
    // update origin
    origin_ = ex.ll * ts;

    // get tiling
    math::Size2i tiling(size(ex));

    // create raster mask for all lods
    for (auto lod : lodRange) {
        masks_.emplace_back(tiling, RasterMask::EMPTY);

        // half the tile
        ts /= 2;

        // double tile count
        tiling.width *= 2;
        tiling.height *= 2;

        // fill in old data
        fill(lod, *other);
    }
}

bool TileIndex::exists(const TileId &tileId) const
{
    const auto *m(mask(tileId.lod));
    if (!m) { return false; }

    // tile size
    auto ts(tileSize(baseTileSize_, tileId.lod));

    // query mask
    return m->get((tileId.easting - origin_(0)) / ts
                  , (tileId.northing - origin_(1)) / ts);
}

void TileIndex::fill(const Metadata &metadata)
{
    for (const auto &node : metadata) {
        const auto &tileId(node.first);

        auto *m(mask(tileId.lod));
        if (!m) {
            // lod out of interest
            continue;
        }

        // tile size
        auto ts(tileSize(baseTileSize_, tileId.lod));

        // set/unset tile presence
        m->set((tileId.easting - origin_(0)) / ts
               , (tileId.northing - origin_(1)) / ts
               , node.second.exists());
    }
}

void TileIndex::fill(Lod lod, const TileIndex &other)
{
    // find old and new masks
    const auto *oldMask(other.mask(lod));
    if (!oldMask) { return; }

    auto *newMask(mask(lod));
    if (!newMask) { return; }

    // calculate origin difference in tiles at given lod
    auto ts(tileSize(baseTileSize_, lod));

    math::Size2 diff((origin_(0) - other.origin_(0)) / ts
                     , (origin_(1) - other.origin_(1)) / ts);

    auto size(oldMask->dims());
    for (int j(0); j < size.height; ++j) {
        for (int i(0); i < size.width; ++i) {
            if (oldMask->get(i, j)) {
                newMask->set(diff.width + i, diff.height + j);
            }
        }
    }
}

void TileIndex::load(std::istream &f)
{
    using utility::binaryio::read;

    char magic[7];
    read(f, magic);

    if (std::memcmp(magic, TILE_INDEX_IO_MAGIC, sizeof(TILE_INDEX_IO_MAGIC))) {
        LOGTHROW(err2, Error)
            << "TileIndex has wrong magic.";
    }

    uint8_t reserved1;
    read(f, reserved1); // reserved

    int64_t o;
    read(f, o);
    origin_(0) = o;
    read(f, o);
    origin_(1) = o;

    int16_t minLod, size;
    read(f, minLod);
    read(f, size);
    minLod_ = minLod;

    masks_.resize(size);

    for (auto &mask : masks_) {
        mask.load(f);
    }
}

void TileIndex::load(const fs::path &path)
{
    std::ifstream f;
    f.exceptions(std::ifstream::failbit | std::ifstream::badbit);
    f.open(path.string(), std::ifstream::in | std::ifstream::binary);

    load(f);

    f.close();
}

void TileIndex::save(std::ostream &f) const
{
    using utility::binaryio::write;

    write(f, TILE_INDEX_IO_MAGIC); // 7 bytes
    write(f, uint8_t(0)); // reserved

    write(f, int64_t(origin_(0)));
    write(f, int64_t(origin_(1)));

    write(f, int16_t(minLod_));
    write(f, int16_t(masks_.size()));

    // save lod-mask mapping
    for (const auto &mask : masks_) {
        mask.dump(f);
    }
}

void TileIndex::save(const fs::path &path) const
{
    utility::ofstreambuf f;
    f.exceptions(std::ifstream::failbit | std::ifstream::badbit);
    f.open(path.string(), std::ifstream::out | std::ifstream::trunc);

    save(f);

    f.close();
}

void TileIndex::growUp()
{
    // TODO: implement me
}

void TileIndex::growDown()
{
    // TODO: implement me
}

TileIndex unite(const Alignment &alignment
                , const std::vector<const TileIndex*> &tis)
{
    // handle special cases
    switch (tis.size()) {
    case 0: return {};
    case 1: return *tis.front();
    }

    // calculate basic parameters
    const auto &front(*tis.front());
    auto baseTileSize(front.baseTileSize());
    auto lodRange(front.lodRange());
    auto extents(front.extents());
    for (const auto *ti : tis) {
        extents = unite(extents, ti->extents());
        lodRange = unite(lodRange, ti->lodRange());
    }

    // result tile index
    TileIndex out(alignment, baseTileSize, extents, lodRange);

    // fill in targets
    for (const auto *ti : tis) {
        for (auto lod : lodRange) {
            out.fill(lod, *ti);
        }
    }

    // done
    return out;
}

} } // namespace vadstena::tilestorage
