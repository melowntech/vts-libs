#ifndef vtslibs_vts_geomextents_hpp
#define vtslibs_vts_geomextents_hpp

#include <limits>

#include <boost/variant.hpp>

#include "math/geometry_core.hpp"

#include "half/half.hpp"

#include "../storage/range.hpp"

typedef half_float::half hfloat;

namespace vtslibs { namespace vts {

struct GeomExtents {
    typedef storage::Range<float> ZRange;

    /** (SDS) height range.
     */
    ZRange z;

    /** (SDS) surrogate value (i.e. representative height of tile)
     */
    float surrogate;

    GeomExtents()
        : z(ZRange::emptyRange())
        , surrogate(-std::numeric_limits<float>::infinity())
    {}

    GeomExtents(float min, float max, float surrogate)
        : z(min, max), surrogate(surrogate)
    {}
};

inline bool empty(const GeomExtents &ge) { return ge.z.empty(); }

inline void update(GeomExtents &ge, float value) {
    update(ge.z, value);
}

inline void update(GeomExtents &ge, const GeomExtents &update) {
    ge.z = unite(ge.z, update.z);
}

} } // namespace vtslibs::vts

#endif // vtslibs_vts_geomextents_hpp

