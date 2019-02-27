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
#ifndef vtslibs_vts0_tileop_hpp_included_
#define vtslibs_vts0_tileop_hpp_included_

#include "../storage/filetypes.hpp"

#include "basetypes.hpp"
#include "properties.hpp"
#include "types.hpp"

namespace vtslibs { namespace vts0 {

using storage::TileFile;
using storage::File;

TileId fromAlignment(const Properties &properties, const TileId &tileId);

TileId parent(const TileId &tileId);

Children children(const TileId &tileId);

bool isMetatile(const LodLevels &levels, const TileId &tile);

Lod deltaDown(const LodLevels &levels, Lod lod);

/** Check whether super tile is above (or exactly the same tile) as tile.
 */
bool above(const TileId &tile, const TileId &super);

int child(const TileId &tileId);

bool in(const LodRange &range, const TileId &tileId);

/** Calculates area of tile's mesh (m^2) and atlas (pixel^2).
 */
std::pair<double, double> area(const Tile &tile);

std::string asFilename(const TileId &tileId, TileFile type);

bool fromFilename(TileId &tileId, TileFile &type
                  , const std::string &str
                  , std::string::size_type offset = 0);

TileId findMetatile(const LodLevels &metaLevels, TileId tileId);

math::Size2f tileSize(const Properties &prop, Lod lod);

math::Size2f tileSize(const math::Extents2 &rootExtents, Lod lod);

std::size_t tileCount(Lod lod);

TileId fromLl(const Properties &prop, Lod lod, const math::Point2 &ll);

math::Extents2 aligned(const Properties &prop, Lod lod
                       , math::Extents2 in);

math::Extents2 extents(const Properties &prop, const TileId &tileId);

// inline stuff

inline TileId parent(const TileId &tileId)
{
    return TileId(tileId.lod - 1, tileId.x >> 1, tileId.y >> 1);
}

inline Children children(const TileId &tileId)
{
    TileId base(tileId.lod + 1, tileId.x << 1, tileId.y << 1);

    // children must be from lower-left due to adapter
    return {{
        { base.lod, base.x, base.y + 1 }        // lower-left
        , { base.lod, base.x + 1, base.y + 1 }  // lower-right
        , base                                  // upper-left
        , { base.lod, base.x + 1, base.y }      // upper-right
    }};
}

inline bool isMetatile(const LodLevels &levels, const TileId &tile)
{
    return !(std::abs(tile.lod - levels.lod) % levels.delta);
}

inline Lod deltaDown(const LodLevels &levels, Lod lod)
{
    Lod res(lod + 1);
    while (std::abs(res - levels.lod) % levels.delta) {
        ++res;
    }
    return res;
}

inline int child(const TileId &tileId)
{
    return (tileId.x & 1l) + ((tileId.y & 1l) << 1);
}

inline bool in(const LodRange &range, const TileId &tileId)
{
    return (tileId.lod >= range.min) && (tileId.lod <= range.max);
}

inline TileId findMetatile(const LodLevels &metaLevels, TileId tileId)
{
    while ((tileId.lod > 0) && !isMetatile(metaLevels, tileId)) {
        tileId = parent(tileId);
    }
    return tileId;
}


inline math::Size2f tileSize(const Properties &prop, Lod lod)
{
    return tileSize(prop.extents, lod);
}

inline std::size_t tileCount(Lod lod)
{
    return std::size_t(1) << lod;
}

} } // namespace vtslibs::vts0

#endif // vtslibs_vts0_tileop_hpp_included_
