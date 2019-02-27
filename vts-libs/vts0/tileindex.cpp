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
#include <boost/filesystem.hpp>
#include <boost/format.hpp>

#include <opencv2/highgui/highgui.hpp>

#include "dbglog/dbglog.hpp"
#include "utility/binaryio.hpp"

#include "imgproc/rastermask/cvmat.hpp"

#include "../storage/error.hpp"

#include "tileindex.hpp"
#include "tileop.hpp"
#include "io.hpp"
#include "tileindex-io.hpp"

namespace vtslibs { namespace vts0 {

namespace fs = boost::filesystem;

namespace {
    const char TILE_INDEX_IO_MAGIC[7] = { 'T', 'I', 'L', 'E', 'I', 'D', 'X' };
}

TileIndex::TileIndex(const TileIndex &other)
    : minLod_(other.minLod_)
    , masks_(other.masks_)
{
}

TileIndex::TileIndex(const TileIndex &other, ShallowCopy)
    : minLod_(other.minLod_)
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

        // double tile count at next lod
        tiling.width <<= 1;
        tiling.height <<= 1;

        // fill in old data (if exists)
        if (other && !noFill) {
            fill(lod, *other);
        };
    }
}

void TileIndex::fill(Lod lod, const TileIndex &other)
{
    // find old and new masks
    const auto *oldMask(other.mask(lod));
    if (!oldMask) { return; }

    auto *newMask(mask(lod));
    if (!newMask) { return; }

    newMask->merge(*oldMask);
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

    newMask->intersect(*oldMask);
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

    newMask->subtract(*oldMask);
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
        LOGTHROW(err2, storage::Error)
            << "TileIndex has wrong magic.";
    }

    uint8_t reserved1;
    read(f, reserved1); // reserved

    int64_t o;
    read(f, o);
    read(f, o);

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

    write(f, int64_t(0));
    write(f, int64_t(1));

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

TileIndex& TileIndex::makeFull()
{
    // nothing to grow
    if (masks_.empty()) { return *this; }
    auto i(masks_.begin());
    math::Size2i tiling(1, 1);
    for (Lod l(0); l < minLod_; ++l, ++i) {
        i = masks_.insert(i, RasterMask(tiling, RasterMask::EMPTY));

        // double tile count at next lod
        tiling.width <<= 1;
        tiling.height <<= 1;
    }

    minLod_ = 0;
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

TileIndex unite(const TileIndices &tis, const Bootstrap &bootstrap)
{
    LOG(info2) << "unite: " << tis.size() << " sets";

    // empty tile set -> nothing
    if (tis.empty()) {
        // just use bootstrap
        // TODO: check bootstrap validity
        return TileIndex(bootstrap.lodRange());
    }

    auto lodRange(bootstrap.lodRange());
    for (const auto *ti : tis) {
        lodRange = unite(lodRange, ti->lodRange());
    }

    LOG(info1) << "unite: lodRange: " << lodRange;

    // result tile index
    TileIndex out(lodRange);

    // fill in targets
    for (const auto *ti : tis) {
        out.fill(*ti);
    }

    // done
    return out;
}

namespace {

template <typename Op>
TileIndex bitop(const TileIndex &l, const TileIndex &r
                , const Bootstrap &bootstrap
                , const Op &op, const char *opName)
{
    LOG(debug) << "Performing " << opName << "(" << &l << ", " << &r << ").";
    if (l.empty() && r.empty()) { return {}; }

    auto lodRange(unite(unite(bootstrap.lodRange()
                              , l.lodRange()), r.lodRange()));
    if (lodRange.empty()) {
        LOG(debug) << opName << ": empty LOD range; nothing to do.";
        return {};
    }

    LOG(debug) << "(" << opName << ") l: " << l;
    LOG(debug) << "(" << opName << ") r: " << r;

    LOG(debug) << "(" << opName << ") lodRange: " << lodRange;

    // result tile index (initialize with first tile index)
    TileIndex out(lodRange, &l);

    // inplace operation
    op(out, r);

    // done
    return out;
}

} // namespace

TileIndex unite(const TileIndex &l, const TileIndex &r
                , const Bootstrap &bootstrap)
{
    return bitop(l, r, bootstrap
                 , [](TileIndex &out, const TileIndex &in) {
                     out.fill(in);
                 }, "unite");
}

TileIndex intersect(const TileIndex &l, const TileIndex &r
                    , const Bootstrap &bootstrap)
{
    return bitop(l, r, bootstrap
                 , [](TileIndex &out, const TileIndex &in) {
                     out.intersect(in);
                 }, "intersect");
}

TileIndex difference(const TileIndex &l, const TileIndex &r
                     , const Bootstrap &bootstrap)
{
    return bitop(l, r, bootstrap
                 , [](TileIndex &out, const TileIndex &in) {
                     out.subtract(in);
                 }, "difference");
}

} } // namespace vtslibs::vts0
