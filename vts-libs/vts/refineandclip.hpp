#ifndef vadstena_libs_vts_refineandclip_hpp
#define vadstena_libs_vts_refineandclip_hpp

#include <functional>

#include "./mesh.hpp"
#include "./basetypes.hpp"

namespace vadstena { namespace vts {

/** Mesh enhanced with projected data.
 */
struct EnhancedSubMesh {
    SubMesh mesh;
    math::Points3d projected;
};

/** Converts projected vertex (whatever it means) to real world coordinates
 *  and to (normalized) external texture coordinates.
 */
struct MeshVertexConvertor {
    /** Convert vertex from projected space to physical space.
     */
    virtual math::Point3d vertex(const math::Point3d &v) const = 0;

    /** Get vertex undulation.
     */
    virtual double undulation(const math::Point3d &v) const = 0;

    /** Convert vertex from projected space to external texture coordinates.
     */
    virtual math::Point2d etc(const math::Point3d &v) const = 0;

    /** Convert original extenral texture coordinates to proper texture
     *  coordinates.
     */
    virtual math::Point2d etc(const math::Point2d &v) const = 0;

    virtual ~MeshVertexConvertor() {}
};

typedef std::function<math::Point3d(const math::Point3d&)> MeshUnproject;

EnhancedSubMesh
refineAndClip(const EnhancedSubMesh &mesh
              , const math::Extents2 &projectedExtents, Lod lodDiff
              , const MeshVertexConvertor &convertor);

} } // namespace vadstena::vts

#endif // vadstena_libs_vts_refineandclip_hpp
