
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
#ifndef vtslibs_vts_meshop_hpp
#define vtslibs_vts_meshop_hpp

#include <functional>

#include "mesh.hpp"
#include "atlas.hpp"
#include "opencv/atlas.hpp"
#include "basetypes.hpp"

namespace vtslibs { namespace vts {

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

typedef std::vector<unsigned int> FaceOriginList;

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
     *  Defaults to no-op
     *
     *  \param current current number of faces in clipped mesh before refinement
     *  \return number of faces to refine mesh to
     */
    virtual std::size_t refineToFaceCount(std::size_t current) const {
        return current;
    }

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
 * \param mesh submesh
 * \param projectedExtents extents in projected space
 * \param mask optional vertex mask (masked out vertices are removed)
 * \param faceOrigin vector of indices to original mesh faces, optional
 * \return clipped mesh
 */
SubMesh clip(const SubMesh &mesh
             , const math::Extents2 &projectedExtents
             , const VertexMask &mask = VertexMask()
             , FaceOriginList *faceOrigin = nullptr);

/** Simple interface to clip mesh that is not in projected space (i.e. spatial
 *  division system).
 *
 * \param mesh submesh
 * \param projected vertices in projected space
 * \param projectedExtents extents in projected space
 * \param mask optional vertex mask (masked out vertices are removed)
 * \param faceOrigin vector of indices to original mesh faces, optional
 * \return clipped mesh
 */
SubMesh clip(const SubMesh &mesh, const math::Points3d &projected
             , const math::Extents2 &projectedExtents
             , const VertexMask &mask = VertexMask()
             , FaceOriginList *faceOrigin = nullptr);

/** Simple interface to clip mesh that is in projected space (i.e. spatial
 *  division system).
 *
 * \param mesh submesh
 * \param projectedVerticalExtent vertical extent in projected space
 * \param mask optional vertex mask (masked out vertices are removed)
 * \return clipped mesh
 */
SubMesh clip(const SubMesh &mesh
             , const math::Extent &projectedVerticalExtent
             , const VertexMask &mask = VertexMask());

/** Simple interface to clip mesh that is not in projected space (i.e. spatial
 *  division system).
 *
 * \param mesh submesh
 * \param projected vertices in projected space
 * \param projectedVerticalExtent vertical extent in projected space
 * \param mask optional vertex mask (masked out vertices are removed)
 * \param faceOrigin vector of indices to original mesh faces, optional
 * \return clipped mesh
 */
SubMesh clip(const SubMesh &mesh, const math::Points3d &projected
             , const math::Extent &projectedVerticalExtent
             , const VertexMask &mask = VertexMask()
             , FaceOriginList *faceOrigin = nullptr);

/** Full interface to clip mesh .
 *
 * \param mesh submesh
 * \param projected vertices in projected space
 * \param projectedExtents extents in projected space
 * \param mask optional vertex mask (masked out vertices are removed)
 * \return clipped enhanced mesh
 */
EnhancedSubMesh clip(const SubMesh &mesh, const math::Points3d &projected
                     , const math::Extents2 &projectedExtents
                     , const MeshVertexConvertor &convertor
                     , const VertexMask &mask = VertexMask()
                     , FaceOriginList *faceOrigin = nullptr);

/** Options for submesh merging.
 */
struct SubmeshMergeOptions {
    enum class AtlasPacking {
        legacy // old texturing mode
        , progressive // new texturing mode
    };

    /** Atlas packing style.
     */
    AtlasPacking atlasPacking;

    /** Maximum number of vertices per one submesh. Use 0 for safe default.
     */
    std::size_t maxVertexCount;

    /** Maximum number of face per one submesh. Use 0 for safe default.
     */
    std::size_t maxFaceCount;

    /** Each submesh has the same mesh and texture faces if set. Useful when
     **  mesh is intended to be used in system that cannot use different indices
     **  in vertices and texture coordinates. Can be used by OpenGL.
     */
    bool sharedFaces;

    SubmeshMergeOptions()
        : atlasPacking(AtlasPacking::progressive)
        , maxVertexCount(), maxFaceCount(), sharedFaces(false)
    {}
};

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
               , const RawAtlas::pointer &atlas, int textureQuality
               , const SubmeshMergeOptions &options = SubmeshMergeOptions());

/** Tries to merge submeshes.
 *  Returns original if no merge cannot be performed.
 *  Only submeshes from same source are merged.
 *  Atlas is repacked when merging textured submeshes.
 *  In case of atlas repacking, returned atlas is image based one.
 *
 *  This version uses hybrid atlas.
 *
 * \param tileId ID of tile this mesh belongs to (info only)
 * \param mesh mesh to compact
 * \param atlas meshe's atlas
 * \param textureQuality JPEG quality (0-100)
 */
std::tuple<Mesh::pointer, Atlas::pointer>
mergeSubmeshes(const TileId &tileId, const Mesh::pointer &mesh
               , const opencv::HybridAtlas::pointer &atlas
               , int textureQuality
               , const SubmeshMergeOptions &options = SubmeshMergeOptions());

/** Compute enhanced submesh area.
 */
SubMeshArea area(const EnhancedSubMesh &submesh, const VertexMask &mask);

/** Compute enhanced mesh area.
 */
MeshArea area(const EnhancedSubMesh::list &mesh, const VertexMasks &masks);

/** Skirting callback. Returns skirt vector to be added to given vertex.
 */
typedef std::function<math::Point3d(math::Point3d)> SkirtVectorCallback;

/** Adds skirt to given mesh.
 */
void addSkirt(EnhancedSubMesh &mesh
              , const MeshVertexConvertor &convertor
              , const SkirtVectorCallback &skirtVector);

/** Makes mesh optimally represented: no duplicity.
 */
SubMesh optimize(SubMesh mesh);

SubMesh makeSharedFaces(const SubMesh &mesh);

// inlines

inline SubMesh clip(const SubMesh &mesh
                    , const math::Extents2 &projectedExtents
                    , const VertexMask &mask
                    , FaceOriginList *faceOrigin)
{
    return clip(mesh, mesh.vertices, projectedExtents, mask, faceOrigin);
}

inline SubMesh clip(const SubMesh &mesh
                    , const math::Extent &projectedVerticalExtent
                    , const VertexMask &mask)
{
    return clip(mesh, mesh.vertices, projectedVerticalExtent, mask);
}

} } // namespace vtslibs::vts

#endif // vtslibs_vts_meshop_hpp
