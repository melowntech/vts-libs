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
#include "./tileindex-io.hpp"

namespace vadstena { namespace vts {

namespace fs = boost::filesystem;

namespace {
    const char TILE_INDEX_IO_MAGIC[7] = { 'T', 'I', 'L', 'E', 'I', 'D', 'X' };
}

TileIndex::TileIndex(const TileIndex &other)
    : origin_(other.origin_)
    , minLod_(other.minLod_)
    , masks_(other.masks_)
{
}

TileIndex::TileIndex(const TileIndex &other, DeepCopy)
    : origin_(other.origin_)
    , minLod_(other.minLod_)
{
    masks_.reserve(other.masks_.size());
    for (const auto &mask : other.masks_) {
        masks_.emplace_back(mask, RasterMask::EMPTY);
    }
}

TileIndex::TileIndex(LodRange lodRange
                     , const TileIndex *other, bool noFill)
{
    // include old definition if non-empty
    if (other && !other->empty()) {
        // something present in on-disk data
        lodRange = unite(lodRange, other->lodRange());
    }

    // set minimum LOD
    minLod_ = lodRange.min;

    // tiling for lowest lod
    math::Size2i tiling(1 << lodRange.min, 1 << lodRange.min);

    // fill in masks
    for (auto lod : lodRange) {
        masks_.emplace_back(tiling, RasterMask::EMPTY);

        LOG(info4) << lod << ": " << tiling;

        // double tile count at next lod
        tiling.width <<= 1;
        tiling.height <<= 1;

        // fill in old data (if exists)
        if (other && !noFill) {
            fill(lod, *other);
        };
    }

    LOG(info4) << "Created TileIndex(" << this->lodRange() << ")";
}

void TileIndex::fill(Lod lod, const TileIndex &other)
{
    // find old and new masks
    const auto *oldMask(other.mask(lod));
    if (!oldMask) { return; }

    auto *newMask(mask(lod));
    if (!newMask) { return; }

    math::Size2 diff((other.origin_(0) - origin_(0))
                     , (other.origin_(1) - origin_(1)));

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

    int16_t minLod(0), size(0);
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

namespace {

double pixelSize(const RasterMask &mask, const long maxArea)
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

} // namespace

void dumpAsImages(const fs::path &path, const TileIndex &ti
                  , const long maxArea)
{
    LOG(info2) << "Dumping tileIndex as image stack at " << path << ".";
    create_directories(path);

    if (ti.masks().empty()) { return; }

    auto lod(ti.lodRange().max);
    const auto &masks(ti.masks());

    for (auto imasks(masks.rbegin()), emasks(masks.rend());
         imasks != emasks; ++imasks)
    {
        LOG(info1) << "Dumping lod " << lod;
        // rasterize and dump
        auto file(path / str(boost::format("%02d.png") % lod));
        cv::Mat mat(asCvMat(*imasks, pixelSize(*imasks, maxArea)));
        imwrite(file.string().c_str(), mat);

        // next level
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

} } // namespace vadstena::vts
