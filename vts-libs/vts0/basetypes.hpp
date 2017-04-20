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
/**
 * \file basertypes.hpp
 * \author Vaclav Blazek <vaclav.blazek@citationtech.net>
 *
 * VTS base types.
 */

#ifndef vtslibs_vts0_basetypes_hpp_included_
#define vtslibs_vts0_basetypes_hpp_included_

#include <string>

#include "../storage/lod.hpp"
#include "../storage/range.hpp"

#include "math/geometry_core.hpp"

namespace vtslibs { namespace vts0 {

using storage::Lod;
using storage::Range;
using storage::LodRange;

/** Tile identifier (index in 3D space): LOD + tile index from upper-left corner
 *  in tile grid.
 */
struct TileId {
    Lod lod;
    long x;
    long y;

    bool operator<(const TileId &tid) const;
    bool operator==(const TileId &tid) const;
    bool operator!=(const TileId &tid) const { return !operator==(tid); }

    TileId(Lod lod = 0, long x = 0, long y = 0)
        : lod(lod), x(x), y(y)
    {}
};

typedef math::Size2_<long> Size2l;

typedef math::Point2_<long> Point2l;
typedef std::vector<Point2l> Points2l;

typedef Point2l Alignment;

typedef math::Extents2_<long> Extents;

typedef std::array<TileId, 4> Children;

/** Lod levels.
 */
struct LodLevels {
    Lod lod;      //!< reference lod
    Lod delta;    //!< lod step

    LodLevels() : lod(), delta() {}
    LodLevels(Lod lod, Lod delta) : lod(lod), delta(delta) {}
};

/** Open mode
 */
enum class OpenMode {
    readOnly     //!< only getters are allowed
    , readWrite  //!< both getters and setters are allowed
};

enum class CreateMode {
    failIfExists //!< creation fails if tile set/storage already exists
    , overwrite  //!< existing tile set/storage is replace with new one
};

// inline stuff

inline bool TileId::operator<(const TileId &tileId) const
{
    if (lod < tileId.lod) { return true; }
    else if (tileId.lod < lod) { return false; }

    if (x < tileId.x) { return true; }
    else if (tileId.x < x) { return false; }

    return y < tileId.y;
}

inline bool TileId::operator==(const TileId &tid) const
{
    return ((lod == tid.lod)
            && (x == tid.x)
            && (y == tid.y));
}

inline bool operator==(const LodLevels &l, const LodLevels &r)
{
    // deltas must be same
    if (l.delta != r.delta) { return false; }
    // difference between reference lods must be divisible by delta
    if (std::abs(l.lod - r.lod) % l.delta) { return false; }
    return true;
}

inline bool operator!=(const LodLevels &l, const LodLevels &r)
{
    return !operator==(l, r);
}

inline Point2l point(const TileId &tid)
{
    return { tid.x, tid.y };
}

} } // namespace vtslibs::vts0

#endif // vtslibs_vts0_basetypes_hpp_included_
