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
#ifndef vtslibs_vts0_tileindex_hpp_included_
#define vtslibs_vts0_tileindex_hpp_included_

#include <map>

#include <boost/filesystem/path.hpp>

#include "imgproc/rastermask.hpp"

#include "maskfwd.hpp"
#include "basetypes.hpp"
#include "tileop.hpp"

namespace vtslibs { namespace vts0 {

class TileIndex {
public:
    TileIndex() : minLod_() {}

    TileIndex(LodRange lodRange, const TileIndex *other = nullptr
              , bool noFill = false);

    TileIndex(const TileIndex &other);

    struct ShallowCopy {};
    TileIndex(const TileIndex &other, ShallowCopy);

    typedef std::vector<RasterMask> Masks;

    void load(std::istream &is);
    void load(const boost::filesystem::path &path);

    void save(std::ostream &os) const;
    void save(const boost::filesystem::path &path) const;

    bool exists(const TileId &tileId) const;

    void fill(Lod lod, const TileIndex &other);

    void fill(const TileIndex &other);

    void intersect(Lod lod, const TileIndex &other);

    void intersect(const TileIndex &other);

    void subtract(Lod lod, const TileIndex &other);

    void subtract(const TileIndex &other);

    void set(const TileId &tileId, bool value = true);

    TileId tileId(Lod lod, long x, long y) const;

    Size2l rasterSize(Lod lod) const;

    bool empty() const;

    Lod minLod() const { return minLod_; }

    Lod maxLod() const;

    LodRange lodRange() const;

    const Masks& masks() const { return masks_; };

    /** Clears lod content.
     */
    void clear(Lod lod);

    /** Returns count of tiles in the index.
     */
    std::size_t count() const;

    const RasterMask* mask(Lod lod) const;

    /** Converts given index to index from given reference in space.
     */
    TileId fromReference(const Point2l &reference, const TileId &index)
        const;

    Point2l fromReference(const Point2l &reference, Lod lod
                          , const Point2l &point)  const;

    /** Grow-up operator:
     *  existing tile -> mark parent and its all 4 children as existing
     */
    TileIndex& growUp();

    /** Grow-down operator:
     *  existing tile -> mark all its 4 children as existing
     */
    TileIndex& growDown();

    /** Makes every existing tile reachable from top level. In other words: if
     *  tile exists, create all its parents up to the root.
     */
    TileIndex& makeComplete();

    TileIndex& invert();

    /** Add LODs from root to first existing LOD.
     */
    TileIndex& makeFull();

private:
    RasterMask* mask(Lod lod);

    Lod minLod_;
    Masks masks_;
};

typedef std::vector<const TileIndex*> TileIndices;

/** Dump tile indes as set of images.
 */
void dumpAsImages(const boost::filesystem::path &path, const TileIndex &ti
                  , const long maxArea = 1 << 26);

class Bootstrap {
public:
    Bootstrap() : lodRange_(LodRange::emptyRange()) {}
    Bootstrap(const LodRange &lodRange) : lodRange_(lodRange) {}
    Bootstrap(const TileIndex &ti) : lodRange_(ti.lodRange()) {}

    const LodRange& lodRange() const { return lodRange_; }
    Bootstrap& lodRange(const LodRange &lodRange) {
        lodRange_ = lodRange;
        return *this;
    }

private:
    LodRange lodRange_;
};

TileIndex unite(const TileIndices &tis
                , const Bootstrap &bootstrap = Bootstrap());

TileIndex unite(const TileIndex &l, const TileIndex &r
                , const Bootstrap &bootstrap = Bootstrap());

TileIndex intersect(const TileIndex &l, const TileIndex &r
                    , const Bootstrap &bootstrap = Bootstrap());

TileIndex difference(const TileIndex &l, const TileIndex &r
                     , const Bootstrap &bootstrap = Bootstrap());

// inline stuff

inline bool TileIndex::empty() const
{
    return masks_.empty();
}

inline Lod TileIndex::maxLod() const
{
    return minLod_ + masks_.size() - 1;
}

inline LodRange TileIndex::lodRange() const
{
    if (empty()) { return LodRange::emptyRange(); }
    return { minLod_, maxLod() };
}

inline const RasterMask* TileIndex::mask(Lod lod) const
{
    auto idx(lod - minLod_);
    if ((idx < 0) || (idx >= int(masks_.size()))) {
        return nullptr;
    }

    // get mask
    return &masks_[idx];
}

inline RasterMask* TileIndex::mask(Lod lod)
{
    auto idx(lod - minLod_);
    if ((idx < 0) || (idx >= int(masks_.size()))) {
        return nullptr;
    }

    // get mask
    return &masks_[idx];
}

inline Size2l TileIndex::rasterSize(Lod lod) const
{
    const auto *m(mask(lod));
    if (!m) { return {}; }
    auto d(m->dims());
    return Size2l(d.width, d.height);
}

inline bool TileIndex::exists(const TileId &tileId) const
{
    const auto *m(mask(tileId.lod));
    if (!m) { return false; }
    return m->get(tileId.x, tileId.y);
}

inline TileId TileIndex::tileId(Lod lod, long x, long y) const
{
    return { lod, x, y };
}

inline void TileIndex::set(const TileId &tileId, bool value)
{
    if (auto *m = mask(tileId.lod)) {
        m->set(tileId.x, tileId.y, value);
    }
}

/** Traverses tile index and calls op(Index) for each existing tile.
 */
template <typename Op>
inline void traverse(const TileIndex &ti, const Op &op)
{
    auto lod(ti.minLod());
    for (const auto &mask : ti.masks()) {
        mask.forEach([&](long x, long y, bool) {
                op(TileId(lod, x, y));
            }, RasterMask::Filter::white);
        ++lod;
    }
}

/** Traverses tile index and calls op(Index) for each existing tile.
 */
template <typename Op>
inline void traverse(Lod lod, const TileIndex &ti, const Op &op)
{
    const auto *mask(ti.mask(lod));
    if (!mask) { return; }

    mask->forEach([&](long x, long y, bool)
    {
        op(TileId(lod, x, y));
    }, RasterMask::Filter::white);
}

/** Traverses tile index and calls op(TileId) for each existing tile.
 */
template <typename Op>
inline void traverseTiles(const TileIndex &ti, const Op &op)
{
    return traverse(ti, op);
}

} } // namespace vtslibs::vts0

#endif // vtslibs_vts0_tileindex_hpp_included_
