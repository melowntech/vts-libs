#ifndef vadstena_libs_vts_refineandclip_hpp
#define vadstena_libs_vts_refineandclip_hpp


#include "./mesh.hpp"
#include "./basetypes.hpp"

namespace vadstena { namespace vts {

/** Mesh enhanced with projected data.
 */
struct EnhancedSubMesh {
    SubMesh mesh;
    math::Points3d projected;
};

EnhancedSubMesh refineAndClip(const EnhancedSubMesh &mesh
                              , const math::Extents2 &projectedExtents
                              , Lod lodDiff);

} } // namespace vadstena::vts

#endif // vadstena_libs_vts_refineandclip_hpp
