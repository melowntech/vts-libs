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

#ifndef vtslibs_vts_basetypes3_hpp_included_
#define vtslibs_vts_basetypes3_hpp_included_

#include "basetypes.hpp"

namespace vtslibs { namespace vts {

/** Extended tile ID in 3-dimensional division.
 *
 * Protected inheritance to prevent implicit upcast.
 */
struct TileId3 : protected TileId {
    using TileId::lod;
    using TileId::x;
    using TileId::y;
    TileId::index_type z;

    TileId3(Lod lod = 0, index_type x = 0, index_type y = 0
            , index_type z = 0)
        : TileId(lod, x, y), z(z)
    {}

    explicit TileId3(const TileId &tileId, index_type z = 0)
        : TileId(tileId), z(z)
    {}

    // "explicit" upcast
    TileId& tileId() { return *this; }
    const TileId& tileId() const { return *this; }

    bool operator<(const TileId3 &tid) const;
    bool operator==(const TileId3 &tid) const;
    bool operator!=(const TileId3 &tid) const;

    typedef std::vector<TileId3> list;
};

struct Child3 : TileId3 {
    unsigned int index;

    Child3(const TileId3 &tileId = TileId3(), unsigned int index = 0)
        : TileId3(tileId), index(index)
    {}

    Child3(Lod lod, unsigned int x, unsigned int y, unsigned int z
           , unsigned int index)
        : TileId3(lod, x, y, z), index(index)
    {}
};

typedef std::array<Child3, 8> Children3;

inline bool TileId3::operator<(const TileId3 &tid) const {
    return (std::tie(static_cast<const TileId&>(*this), z)
            < std::tie(static_cast<const TileId&>(tid), tid.z));
}

inline bool TileId3::operator==(const TileId3 &tid) const {
    return (std::tie(static_cast<const TileId&>(*this), z)
            == std::tie(static_cast<const TileId&>(tid), tid.z));
}

inline bool TileId3::operator!=(const TileId3 &tid) const {
    return !operator==(tid);
}

} } // namespace vtslibs::vts

#endif // vtslibs_vts_basetypes3_hpp_included_
