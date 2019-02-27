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
#ifndef vtslibs_vts_types2d_hpp_included_
#define vtslibs_vts_types2d_hpp_included_

#include <string>

#include "math/geometry_core.hpp"

#include "../registry/referenceframe.hpp"

#include "types.hpp"

namespace vtslibs { namespace vts {

struct Meta2d {
    struct Flag {
        typedef std::uint8_t value_type;
        enum : value_type {
            geometry = 0x80
           , nonmasked = 0x40
           , ophoto = 0x20
           , alien = 0x10

           // no flag set
           , none = 0x00
        };
    };

    static constexpr unsigned int binaryOrder = 8;
    static constexpr unsigned int restMask = ((1 << binaryOrder) - 1);
    static constexpr unsigned int idMask = ~restMask;

    static math::Size2 size() {
        return math::Size2(1 << binaryOrder, 1 << binaryOrder);
    }

    static TileId metaId(const TileId &id) {
        return TileId(id.lod, idMask & id.x, idMask & id.y);
    }

    static bool isMetaId(const TileId &id) {
        return !((id.x & restMask) || (id.y & restMask));
    }

    static TileId localId(const TileId &id) {
        return TileId(id.lod, restMask & id.x, restMask & id.y);
    }
};

struct Mask2d {
    struct Flag {
        typedef std::uint8_t value_type;
        enum : value_type {
            submesh = 0x80
           , ophoto = 0x40

           // no flag set
           , none = 0x00
        };
    };

    static math::Size2 size() { return math::Size2(256, 258); }
    static math::Size2 maskSize() { return math::Size2(256, 256); }
    static constexpr int flagRow = 256;
    static constexpr int surfaceRow = 257;
};

struct CreditTile {
    registry::Credits credits;

    void update(const CreditTile &other);

    void expand(const registry::Registry &reg = registry::system);
    void expand(const registry::Credit::dict &dict);

    static constexpr unsigned int binaryOrder = Meta2d::binaryOrder;

    static constexpr unsigned int restMask = ((1 << binaryOrder) - 1);
    static constexpr unsigned int idMask = ~restMask;

    static TileId creditsId(const TileId &id) {
        return TileId(id.lod, idMask & id.x, idMask & id.y);
    }

    static bool isCreditId(const TileId &id) {
        return !((id.x & restMask) || (id.y & restMask));
    }
};

// inlines

inline void CreditTile::expand(const registry::Registry &reg)
{
    expand(reg.credits);
}

} } // namespace vtslibs::vts

#endif // vtslibs_vts_types2d_hpp_included_
