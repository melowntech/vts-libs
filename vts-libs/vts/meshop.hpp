#ifndef vadstena_libs_vts_meshop_hpp
#define vadstena_libs_vts_meshop_hpp

#include <functional>

#include "./mesh.hpp"
#include "./atlas.hpp"
#include "./basetypes.hpp"

namespace vadstena { namespace vts {

/** Mesh enhanced with projected data.
 */
struct EnhancedSubMesh {
    SubMesh mesh;
    math::Points3d projected;

    operator bool() const {
        return !mesh.vertices.empty();
    }

    typedef std::vector<EnhancedSubMesh> list;

    EnhancedSubMesh() {}
    EnhancedSubMesh(const SubMesh &m) : mesh(m) {
        projected.reserve(m.vertices.size());
    }

    EnhancedSubMesh(const SubMesh &m, const math::Points3d &projected)
        : mesh(m), projected(projected)
    {}

    struct AllocateProjected {};

    EnhancedSubMesh(const SubMesh &m, AllocateProjected) : mesh(m) {
        projected.resize(m.vertices.size());
    }
};

/** Converts projected vertex (whatever it means) to real world coordinates
 *  and to (normalized) external texture coordinates.
 */
struct MeshVertexConvertor {
    /** Convert vertex from projected space to physical space.
     */
    virtual math::Point3d vertex(const math::Point3d &v) const = 0;

    /** Convert vertex from projected space to external texture coordinates.
     */
    virtual math::Point2d etc(const math::Point3d &v) const = 0;

    /** Convert original external texture coordinates to proper texture
     *  coordinates.
     */
    virtual math::Point2d etc(const math::Point2d &v) const = 0;

    /** Number of faces to refine mesh to.
     *
     *  \param current current number of faces in clipped mesh before refinement
     *  \return number of faces to refine mesh to
     */
    virtual std::size_t refineToFaceCount(std::size_t current) const = 0;

    virtual ~MeshVertexConvertor() {}
};

typedef std::function<math::Point3d(const math::Point3d&)> MeshUnproject;

/** Clips and refines mesh that is in physical coordinates and provides means to
 *  work in projected space (i.e. spatial division system).
 *
 * \param mesh enhanced mesh (mesh + projected vertices)
 * \param projectedExtents extents in projected space
 * \param convertor vertex convertor (for new vertex generation)
 * \param mask optional vertex mask (masked out vertices are removed)
 * \return clipped and refined mesh
 */
EnhancedSubMesh
clipAndRefine(const EnhancedSubMesh &mesh
              , const math::Extents2 &projectedExtents
              , const MeshVertexConvertor &convertor
              , const VertexMask &mask = VertexMask());

/** Simple interface to clip mesh that is in projected space (i.e. spatial
 *  division system).
 *
 * \param projectedMesh mesh in projected space
 * \param projectedExtents extents in projected space
 * \param mask optional vertex mask (masked out vertices are removed)
 * \return clipped mesh
 */
SubMesh clip(const SubMesh &projectedMesh
             , const math::Extents2 &projectedExtents
             , const VertexMask &mask = VertexMask());

/** Tries to merge submeshes.
 *  Returns original if no merge cannot be performed.
 *  Only submeshes from same source are merged.
 *  Atlas is repacked when merging textured submeshes.
 *  In case of atlas repacking, returned atlas is image based one.
 *
 * \param tileId ID of tile this mesh belongs to (info only)
 * \param mesh mesh to compact
 * \param atlas meshe's atlas
 * \param textureQuality JPEG quality (0-100)
 */
std::tuple<Mesh::pointer, Atlas::pointer>
mergeSubmeshes(const TileId &tileId, const Mesh::pointer &mesh
               , const RawAtlas::pointer &atlas, int textureQuality);

/** Compute enhanced submesh area.
 */
SubMeshArea area(const EnhancedSubMesh &submesh, const VertexMask &mask);

/** Compute enhanced mesh area.
 */
MeshArea area(const EnhancedSubMesh::list &mesh, const VertexMasks &masks);

} } // namespace vadstena::vts

#endif // vadstena_libs_vts_meshop_hpp
