#include <boost/filesystem.hpp>
#include <boost/format.hpp>

#include <opencv2/highgui/highgui.hpp>

#include "dbglog/dbglog.hpp"
#include "utility/binaryio.hpp"

#include "imgproc/rastermask/cvmat.hpp"

#include "./tileindex.hpp"
#include "./tileop.hpp"
#include "./error.hpp"
#include "./io.hpp"

namespace vadstena { namespace tilestorage {

namespace fs = boost::filesystem;

namespace {
    const char TILE_INDEX_IO_MAGIC[7] = { 'T', 'I', 'L', 'E', 'I', 'D', 'X' };
}

namespace {

/** Converts coordinate to grid defined by reference point and cell size.
 *  Coordinates not on grid are rounded up.
 */
long gridUp(long value, long origin, long size)
{
    auto oft(value - origin);
    if (oft < 0) {
        return (oft / size);
    }
    return ((oft + (size - 1)) / size);
}

/** Converts coordinate to grid defined by reference point and cell size.
 *  Coordinates not on grid are rounded down.
 */
long gridDown(long value, long origin, long size)
{
    auto oft(value - origin);
    if (oft < 0) {
        return ((oft - (size - 1)) / size);
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
                     , const TileIndex *other, bool noFill)
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

    LOG(info1) << "TileIndex tiling: extents=" << ex
               << ", size=" << tiling << ".";

    // create raster mask for all lods
    for (auto lod : lodRange) {
        masks_.emplace_back(tiling, RasterMask::EMPTY);

        // half the tile
        ts /= 2;

        // double tile count
        tiling.width *= 2;
        tiling.height *= 2;

        // fill in old data (if exists)
        if (other && !noFill) {
            fill(lod, *other);
        };
    }
}

TileIndex::TileIndex(const TileIndex &other)
    : baseTileSize_(other.baseTileSize_)
    , origin_(other.origin_)
    , minLod_(other.minLod_)
    , masks_(other.masks_)
{
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

    math::Size2 diff((other.origin_(0) - origin_(0)) / ts
                     , (other.origin_(1) - origin_(1)) / ts);

    auto nsize(newMask->dims());
    auto size(oldMask->dims());

    if (!(diff.width || diff.height) && (nsize == size)) {
        // same-sized masks at same position -> just merge
        newMask->merge(*oldMask);
        return;
    }

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

    math::Size2 diff((other.origin_(0) - origin_(0)) / ts
                     , (other.origin_(1) - origin_(1)) / ts);

    auto nsize(newMask->dims());
    auto size(oldMask->dims());

    if (!(diff.width || diff.height) && (nsize == size)) {
        // same-sized masks at same position -> just intersect
        newMask->intersect(*oldMask);
        return;
    }

    for (int j(0); j < size.height; ++j) {
        for (int i(0); i < size.width; ++i) {
            if (!oldMask->get(i, j)
                && newMask->get(diff.width + i, diff.height + j))
            {
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

    math::Size2 diff((other.origin_(0) - origin_(0)) / ts
                     , (other.origin_(1) - origin_(1)) / ts);

    auto nsize(newMask->dims());
    auto size(oldMask->dims());

    if (!(diff.width || diff.height) && (nsize == size)) {
        // same-sized masks at same position -> just subtract
        newMask->subtract(*oldMask);
        return;
    }

    if (newMask->empty() || oldMask->empty()) {
        // either empty -> nothing to do
        return;
    }

    for (int j(0); j < size.height; ++j) {
        for (int i(0); i < size.width; ++i) {
            if (oldMask->get(i, j)
                && newMask->get(diff.width + i, diff.height + j))
            {
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
    auto lod(lodRange().max);
    auto cmasks(masks_.rbegin());

    for (auto imasks(cmasks + 1), emasks(masks_.rend());
         imasks != emasks; ++imasks, ++cmasks, --lod)
    {
        LOG(debug) << "gu: " << lod << " -> " << (lod - 1);

        auto &child(*cmasks);
        auto &mask(*imasks);

        // coarsen
        child.coarsen(2);
        mask.merge(child, false);
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
    auto lod(lodRange().min);
    auto pmasks(masks_.begin());

    for (auto imasks(pmasks + 1), emasks(masks_.end());
         imasks != emasks; ++imasks, ++pmasks, ++lod)
    {
        LOG(debug) << "gd: " << lod << " -> " << (lod + 1);
        const auto &parent(*pmasks);
        auto &mask(*imasks);
        // merge in parent mask, ignore its size
        mask.merge(parent, false);
    }

    return *this;
}

TileIndex& TileIndex::makeComplete()
{
    if (masks_.size() < 2) {
        // nothing to grow
        return *this;
    }

    // traverse masks bottom to top
    auto lod(lodRange().max);
    auto cmasks(masks_.rbegin());

    for (auto imasks(cmasks + 1), emasks(masks_.rend());
         imasks != emasks; ++imasks, ++cmasks, --lod)
    {
        LOG(debug) << "gu: " << lod << " -> " << (lod - 1);

        // make copy of child
        RasterMask child(*cmasks);
        auto &mask(*imasks);

        // coarsen child (do not change child!)
        child.coarsen(2);
        // merge in coarsened child -> all parents are set
        mask.merge(child, false);
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
    LOG(info2) << "unite: " << tis.size() << " sets";

    // empty tile set -> nothing
    if (tis.empty()) {
        // just use bootstrap
        // TODO: check bootstrap validity
        return TileIndex(alignment, bootstrap.baseTileSize()
                         , bootstrap.extents(), bootstrap.lodRange());
    }

    // calculate basic parameters
    auto baseTileSize(bootstrap.baseTileSize()
                      ? bootstrap.baseTileSize()
                      : tis.front()->baseTileSize());
    auto lodRange(bootstrap.lodRange());
    auto extents(bootstrap.extents());
    for (const auto *ti : tis) {
        extents = uniteExtents(extents, ti->extents());
        lodRange = unite(lodRange, ti->lodRange());
    }

    LOG(info1) << "unite: lodRange: " << lodRange
               << ", extents: " << extents;

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
                , const Op &op, const char *opName)
{
    if (l.empty() && r.empty()) { return {}; }

    auto baseTileSize(l.empty() ? r.baseTileSize() : l.baseTileSize());
    if (!baseTileSize) { return {}; }

    auto lodRange(unite(unite(bootstrap.lodRange()
                              , l.lodRange()), r.lodRange()));
    if (lodRange.empty()) { return {}; }

    auto extents(uniteExtents(uniteExtents(bootstrap.extents()
                                           , l.extents()), r.extents()));
    if (empty(extents)) { return {}; }

    LOG(debug) << "(" << opName << ") l: " << l;
    LOG(debug) << "(" << opName << ") r: " << r;

    LOG(debug) << "(" << opName << ") lodRange: " << lodRange;
    LOG(debug) << "(" << opName << ") extents: " << extents;

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
                 }, "unite");
}

TileIndex intersect(const Alignment &alignment
                    , const TileIndex &l, const TileIndex &r
                    , const Bootstrap &bootstrap)
{
    return bitop(alignment, l, r, bootstrap
                 , [](TileIndex &out, const TileIndex &in) {
                     out.intersect(in);
                 }, "intersect");
}

TileIndex difference(const Alignment &alignment
                     , const TileIndex &l, const TileIndex &r
                     , const Bootstrap &bootstrap)
{
    return bitop(alignment, l, r, bootstrap
                 , [](TileIndex &out, const TileIndex &in) {
                     out.subtract(in);
                 }, "difference");
}

namespace {

double initialPixelSize(const RasterMask &mask
                                       , const long maxArea)
{
    const auto dims(mask.dims());
    long a(long(dims.width) * long(dims.height));
    if (a <= maxArea) {
        // OK
        return 1.;
    }

    auto scale(std::sqrt(double(maxArea) / double(a)));
    return scale;
}

}

void dumpAsImages(const fs::path &path, const TileIndex &ti
                  , const long maxArea)
{
    LOG(info2) << "Dumping tileIndex as image stack at " << path << ".";
    create_directories(path);

    if (ti.masks().empty()) { return; }

    auto lod(ti.lodRange().max);
    const auto &masks(ti.masks());

    auto pixelSize(initialPixelSize(masks.back(), maxArea));
    for (auto imasks(masks.rbegin()), emasks(masks.rend());
         imasks != emasks; ++imasks)
    {
        LOG(info1) << "Dumping lod " << lod;
        // rasterize and dump
        auto file(path / str(boost::format("%02d.png") % lod));
        cv::Mat mat;
        flip(asCvMat(*imasks, pixelSize), mat, 0);
        imwrite(file.string().c_str(), mat);

        // next level
        pixelSize *= 2;
        --lod;
    }
}

void TileIndex::clear(Lod lod)
{
    auto *m(mask(lod));
    if (!m) { return; }

    *m = RasterMask(m->dims(), RasterMask::InitMode::EMPTY);
}

std::size_t TileIndex::count() const
{
    std::size_t total(0);
    for (const auto &mask : masks_) {
        total += mask.count();
    }
    return total;
}

} } // namespace vadstena::tilestorage
