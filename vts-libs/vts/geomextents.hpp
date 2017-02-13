#ifndef vadstena_libs_vts_geomextents_hpp
#define vadstena_libs_vts_geomextents_hpp

#include <boost/variant.hpp>

#include "math/geometry_core.hpp"

#include "half/half.hpp"

#include "../storage/range.hpp"

typedef half_float::half hfloat;

namespace vadstena { namespace vts {

struct GeomExtents {
    typedef Range<float> ZRange;
    ZRange z;
    float surrogate;

    GeomExtents() : z(), surrogate() {}

    GeomExtents(const math::InvalidExtents&)
        : z(ZRange::emptyRange()), surrogate()
    {}
};

inline bool empty(const GeomExtents &ge) { return ge.z.empty(); }

} } // namespace vadstena::vts

#endif // vadstena_libs_vts_geomextents_hpp
