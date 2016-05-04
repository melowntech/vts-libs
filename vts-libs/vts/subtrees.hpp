#ifndef vadstena_libs_vts_subtrees_hpp_included_
#define vadstena_libs_vts_subtrees_hpp_included_

#include <map>

#include "math/geometry_core.hpp"
#include "geo/srsdef.hpp"

#include "./basetypes.hpp"

namespace vadstena { namespace vts {

struct Subtrees {
    typedef std::map<TileId, TileRange> Ranges;

    Lod lod;
    Ranges ranges;
    TileRange overallRange;

    operator bool() const { return math::valid(overallRange); }
};

/** Breaks rectangular area (in given SRS) into subtrees in given reference
 *  frame at given LOD.
 */
Subtrees findSubtrees(const registry::ReferenceFrame &referenceFrame, Lod lod
                      , const geo::SrsDefinition &srs
                      , const math::Extents2 &extents
                      , int samples = 20);

} } // namespace vadstena::vts

#endif // vadstena_libs_vts_subtrees_hpp_included_
