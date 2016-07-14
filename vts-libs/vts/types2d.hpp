#ifndef vadstena_libs_vts_types2d_hpp_included_
#define vadstena_libs_vts_types2d_hpp_included_

#include <string>

#include "math/geometry_core.hpp"

#include "../registry/referenceframe.hpp"

#include "./types.hpp"

namespace vadstena { namespace vts {

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

    static constexpr unsigned int binaryOrder = 8;

    static constexpr unsigned int restMask = ((1 << binaryOrder) - 1);
    static constexpr unsigned int idMask = ~restMask;

    static TileId creditsId(const TileId &id) {
        return TileId(id.lod, idMask & id.x, idMask & id.y);
    }

    static bool isCreditId(const TileId &id) {
        return !((id.x & restMask) || (id.y & restMask));
    }
};

} } // namespace vadstena::vts

#endif // vadstena_libs_vts_types2d_hpp_included_
