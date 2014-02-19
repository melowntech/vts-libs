#include <boost/filesystem.hpp>
#include <boost/format.hpp>

#include <opencv2/highgui/highgui.hpp>

#include "dbglog/dbglog.hpp"
#include "utility/binaryio.hpp"

#include "imgproc/rastermask/cvmat.hpp"

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
        return (oft / size);
    }

    return ((size - 1 + oft) / size);
}

/** Converts coordinate to grid defined by reference point and cell size.
 *  Coordinates not on grid are rounded down.
 */
long gridDown(long value, long origin, unsigned long size)
{
    auto oft(value - origin);
    if (oft < 0) {
        return ((size - 1 - oft) / size);
    }

    return (oft / size);
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
    origin_ = ts * ex.ll + alignment;

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

void TileIndex::fill(const TileIndex &other)
{
    for (auto lod : lodRange()) {
        fill(lod, other);
    }
}

void TileIndex::intersect(Lod lod, const TileIndex &other)
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
            if (!oldMask->get(i, j) && newMask->get(i, j)) {
                // new set but old unset -> unset
                newMask->set(diff.width + i, diff.height + j
                             , false);
            }
        }
    }
}

void TileIndex::intersect(const TileIndex &other)
{
    for (auto lod : lodRange()) {
        intersect(lod, other);
    }
}

void TileIndex::subtract(Lod lod, const TileIndex &other)
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
            if (oldMask->get(i, j) && newMask->get(i, j)) {
                // new set but old unset -> unset
                newMask->set(diff.width + i, diff.height + j
                             , false);
            }
        }
    }
}

void TileIndex::subtract(const TileIndex &other)
{
    for (auto lod : lodRange()) {
        subtract(lod, other);
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

TileIndex& TileIndex::growUp()
{
    if (masks_.size() < 2) {
        // nothing to grow
        return *this;
    }

    // traverse masks bottom to top
    auto cmasks(masks_.rbegin());
    for (auto imasks(cmasks + 1), emasks(masks_.rend());
         imasks != emasks; ++imasks, ++cmasks)
    {
        auto &child(*cmasks);
        auto &mask(*imasks);

        // rasterize current mask
        auto size(mask.dims());
        for (int j(0), jj(0); j < size.height; ++j, jj += 2) {
            for (int i(0), ii(0); i < size.width; ++i, ii += 2) {
                // if there is any child ensure parent exists and as well as all
                // four children

                if (child.get(ii, jj) || child.get(ii + 1, jj)
                    || child.get(ii, jj + 1) || child.get(ii + 1, jj + 1))
                {
                    // at least one child exists, mark all children
                    child.set(ii, jj);
                    child.set(ii + 1, jj);
                    child.set(ii, jj + 1);
                    child.set(ii + 1, jj + 1);

                    // mark me
                    mask.set(i, j);
                }
            }
        }
    }

    return *this;
}

TileIndex& TileIndex::growDown()
{
    if (masks_.size() < 2) {
        // nothing to grow
        return *this;
    }

    // traverse masks top to bottom
    auto pmasks(masks_.begin());
    for (auto imasks(pmasks + 1), emasks(masks_.end());
         imasks != emasks; ++imasks, ++pmasks)
    {
        const auto &parent(*pmasks);
        auto &mask(*imasks);

        // rasterize parent mask
        auto size(parent.dims());
        for (int j(0), jj(0); j < size.height; ++j, jj += 2) {
            for (int i(0), ii(0); i < size.width; ++i, ii += 2) {
                // if previous tile exists ensure all four children exist
                if (parent.get(i, j)) {
                    mask.set(ii, jj);
                    mask.set(ii + 1, jj);
                    mask.set(ii, jj + 1);
                    mask.set(ii + 1, jj + 1);
                }
            }
        }
    }

    return *this;
}

TileIndex& TileIndex::invert()
{
    // invert all masks
    for (auto imasks(masks_.begin()), emasks(masks_.end());
         imasks != emasks; ++imasks)
    {
        imasks->invert();
    }

    return *this;
}

namespace {

Extents uniteExtents(const Extents &l, const Extents &r)
{
    if (empty(l)) { return r; }
    if (empty(r)) { return l; }
    return unite(l, r);
}

} // namespace

TileIndex unite(const Alignment &alignment
                , const std::vector<const TileIndex*> &tis
                , const Bootstrap &bootstrap)
{
    // handle special cases
    switch (tis.size()) {
    case 0: return {};
    case 1: return *tis.front();
    }

    // calculate basic parameters
    auto baseTileSize(tis.front()->baseTileSize());
    auto lodRange(bootstrap.lodRange());
    auto extents(bootstrap.extents());
    for (const auto *ti : tis) {
        extents = uniteExtents(extents, ti->extents());
        lodRange = unite(lodRange, ti->lodRange());
    }

    LOG(info2) << "lodRange: " << lodRange;
    LOG(info2) << "extents: " << extents;

    // result tile index
    TileIndex out(alignment, baseTileSize, extents, lodRange);

    // fill in targets
    for (const auto *ti : tis) {
        out.fill(*ti);
    }

    // done
    return out;
}

namespace {

template <typename Op>
TileIndex bitop(const Alignment &alignment
                , const TileIndex &l, const TileIndex &r
                , const Bootstrap &bootstrap
                , const Op &op)
{
    auto baseTileSize(l.baseTileSize());
    auto lodRange(unite(unite(bootstrap.lodRange()
                              , l.lodRange()), r.lodRange()));
    if (lodRange.empty()) { return {}; }

    auto extents(uniteExtents(uniteExtents(bootstrap.extents()
                                           , l.extents()), r.extents()));
    if (empty(extents)) { return {}; }

    LOG(info2) << "(unite) lodRange: " << lodRange;
    LOG(info2) << "(unite) extents: " << extents;

    // result tile index (initialize with first tile index)
    TileIndex out(alignment, baseTileSize, extents, lodRange, &l);

    // inplace operation
    op(out, r);

    // done
    return out;
}

} // namespace

TileIndex unite(const Alignment &alignment
                , const TileIndex &l, const TileIndex &r
                , const Bootstrap &bootstrap)
{
    return bitop(alignment, l, r, bootstrap
                 , [](TileIndex &out, const TileIndex &in) {
                     out.fill(in);
                 });
}

TileIndex intersect(const Alignment &alignment
                    , const TileIndex &l, const TileIndex &r
                    , const Bootstrap &bootstrap)
{
    return bitop(alignment, l, r, bootstrap
                 , [](TileIndex &out, const TileIndex &in) {
                     out.intersect(in);
                 });
}

TileIndex subtract(const Alignment &alignment
                   , const TileIndex &l, const TileIndex &r
                   , const Bootstrap &bootstrap)
{
    return bitop(alignment, l, r, bootstrap
                 , [](TileIndex &out, const TileIndex &in) {
                     out.subtract(in);
                 });
}

void dumpAsImages(const fs::path &path, const TileIndex &ti)
{
    create_directories(path);

    auto lod(ti.lodRange().max);
    const auto &masks(ti.masks());
    int pixelSize(1);
    for (auto imasks(masks.rbegin()), emasks(masks.rend());
         imasks != emasks; ++imasks)
    {
        // rasterize and dump
        auto file(path / str(boost::format("%02d.png") % lod));
        auto mat(asCvMat(*imasks, pixelSize));
        imwrite(file.string().c_str(), mat);

        // next level
        pixelSize *= 2;
        --lod;
    }
}

} } // namespace vadstena::tilestorage
