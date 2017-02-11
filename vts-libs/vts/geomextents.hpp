#ifndef vadstena_libs_vts_geomextents_hpp
#define vadstena_libs_vts_geomextents_hpp

#include <boost/variant.hpp>

#include "math/geometry_core.hpp"

#include "vts-libs/storage/range.hpp"

typedef half_float::half hfloat;

namespace vadstena { namespace vts {

struct GeomExtents {
    Range<float> z;
    float surrogate;

    GeomExtents() : z(), surrogate() {}
};

} } // namespace vadstena::vts

#endif // vadstena_libs_vts_geomextents_hpp
