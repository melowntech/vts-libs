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

#ifndef vtslibs_vts_tileop3_hpp_included_
#define vtslibs_vts_tileop3_hpp_included_

#include <new>

#include "math/extent.hpp"

#include "../storage/filetypes.hpp"

#include "basetypes3.hpp"

namespace vtslibs { namespace vts {

TileId3 parent(const TileId3 &tileId, Lod diff = 1);

Children3 children(const TileId3 &tileId);
TileId3 lowestChild(const TileId3 &tileId, Lod diff = 1);

/** Check whether super tile is above (or exactly the same tile) as tile.
 */
bool above(const TileId3 &tile, const TileId3 &super);

int child(const TileId3 &tileId);

bool in(const LodRange &range, const TileId3 &tileId);


TileId3 local(Lod rootLod, const TileId3 &tileId);

/** Makes global tile ID from local tile id and new root
 */
TileId3 global(const TileId3 &root, const TileId3 &localId);

// inlines

inline TileId3 parent(const TileId3 &tileId, Lod diff)
{
    // do not let new id to go above root
    if (diff > tileId.lod) { return {}; }
    return TileId3(tileId.lod - diff, tileId.x >> diff, tileId.y >> diff
                   , tileId.z >> diff);
}

inline Children3 children(const TileId3 &tileId)
{
    TileId3 base(tileId.lod + 1, tileId.x << 1, tileId.y << 1, tileId.y << 2);

    return {{
        // bottom layer
        { base, 0 }                                          // upper-left
        , { base.lod, base.x + 1, base.y, base.z, 1 }        // upper-right
        , { base.lod, base.x, base.y + 1, base.z, 2 }        // lower-left
        , { base.lod, base.x + 1, base.y + 1, base.z, 3}     // lower-right
        // top layer
        , { base.lod, base.x, base.y, base.z + 1, 4 }        // upper-right
        , { base.lod, base.x + 1, base.y, base.z + 1, 5 }    // upper-right
        , { base.lod, base.x, base.y + 1, base.z + 1, 6 }    // lower-left
        , { base.lod, base.x + 1, base.y + 1, base.z + 1, 7} // lower-right
    }};
}

inline TileId3 lowestChild(const TileId3 &tileId, Lod diff)
{
    return TileId3(tileId.lod + diff, tileId.x << diff, tileId.y << diff
                   , tileId.z << diff);
}

inline int child(const TileId3 &tileId)
{
    return (tileId.x & 1l) + ((tileId.y & 1l) << 1) + ((tileId.y & 1l) << 2);
}

inline bool in(const LodRange &range, const TileId3 &tileId)
{
    return (tileId.lod >= range.min) && (tileId.lod <= range.max);
}

inline TileId3 local(Lod rootLod, const TileId3 &tileId)
{
    if (rootLod >= tileId.lod) { return {}; }

    const auto ldiff(tileId.lod - rootLod);
    const auto mask((1 << ldiff) - 1);
    return TileId3(ldiff, tileId.x & mask, tileId.y & mask, tileId.z & mask);
}

inline TileId3 global(const TileId3 &root, const TileId3 &localId)
{
    return TileId3(root.lod + localId.lod
                   , localId.x + (root.x << localId.lod)
                   , localId.y + (root.y << localId.lod)
                   , localId.z + (root.z << localId.lod));
}

} } // namespace vtslibs::vts

#endif // vtslibs_vts_tileop3_hpp_included_
