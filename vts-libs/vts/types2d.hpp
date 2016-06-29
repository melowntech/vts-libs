#ifndef vadstena_libs_vts_types2d_hpp_included_
#define vadstena_libs_vts_types2d_hpp_included_

#include <string>

#include "math/geometry_core.hpp"

namespace vadstena { namespace vts {

struct Meta2d {
    struct Flag {
        typedef std::uint8_t value_type;
        enum : value_type {
            geometry = 0x80
           , full = 0x40
           , ophoto = 0x20

           // no flag set
           , none = 0x00
        };
    };

    static math::Size2 size() { return math::Size2(256, 256); }
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

} } // namespace vadstena::vts

#endif // vadstena_libs_vts_types2d_hpp_included_
