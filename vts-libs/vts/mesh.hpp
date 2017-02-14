#ifndef vadstena_libs_vts_mesh_hpp
#define vadstena_libs_vts_mesh_hpp

#include <cstdint>
#include <iosfwd>
#include <vector>
#include <memory>

#include <boost/filesystem/path.hpp>

#include "utility/enum-io.hpp"
#include "math/geometry_core.hpp"
#include "imgproc/rastermask/quadtree.hpp"

#include "../storage/streams.hpp"

#include "./qtree.hpp"
#include "./multifile.hpp"
#include "./geomextents.hpp"
#include "./csconvertor.hpp"

namespace vadstena { namespace vts {

typedef math::Point3_<std::uint32_t> Point3u32;
typedef Point3u32 Face;
typedef std::vector<Face> Faces;
typedef std::vector<bool> VertexMask;
typedef std::vector<VertexMask> VertexMasks;

// fwd
class Atlas;

struct SubMesh {
    /** Vertices
     */
    math::Points3d vertices;

    /** Internal texture coordinates (empty if none).
     */
    math::Points2d tc;

    /** Per-vertex external texture coordinates (empty if none, otherwise must
     *  have the same size as vertices).
     */
    math::Points2d etc;

    /** Indices to face vertices.
     */
    Faces faces;

    /** Indices to internal texture coordinates (empty if none, otherwise must
     *  have the same size as faces).
     */
    Faces facesTc;

    enum TextureMode { internal, external };

    /** Texturing mode. Member textureLayer is interpreted only when
     *  textureMode == TextureMode::external.
     */
    TextureMode textureMode;

    /** ID of external texture layer. Can be undefined even when texture mode is
     *  external.
     */
    boost::optional<std::uint16_t> textureLayer;

    typedef std::uint8_t SurfaceReference;
    /** Surface reference. One-based. Defaults to 1.
     */
    SurfaceReference surfaceReference;

    /** UV area scaling factor to artificially scale UV mesh area, i.e. to make
     *  it look bigger.
     *
     * NB: this value is stored outside mesh served to the client!
     */
    double uvAreaScale;

    typedef std::vector<SubMesh> list;

    SubMesh()
        : textureMode(TextureMode::internal), surfaceReference(1)
        , uvAreaScale(1.0)
    {}

    /** Clones metadata (texture mod, layer etc.).
     */
    void cloneMetadataInto(SubMesh &dst) const;

    bool empty() const { return vertices.empty(); }

    /** Filter out degenerate faces and unused vertices, return a new submesh.
     */
    SubMesh cleanUp() const;
};

/** Mesh with submeshes and mask.
 */
struct Mesh {
    typedef std::shared_ptr<Mesh> pointer;
    typedef QTree CoverageMask;

    SubMesh::list submeshes;
    CoverageMask coverageMask;

    static constexpr int coverageOrder = 8;

    static math::Size2i coverageSize() {
        return math::Size2i(1 << coverageOrder, 1 << coverageOrder);
    };

    /** Index inside multifile where mesh is stored.
     */
    static constexpr unsigned int meshIndex() { return 0; }

    /** Creates new (emtpty) mesh.
     *
     *  \param fullyCovered specifies how to initialize coverage mask (true ->
     *  full, false -> empty)
     */
    Mesh(bool fullyCovered = true);

    Mesh(const Mesh&) = default;

    /** Copy first N submeshes from source.
     */
    Mesh(const Mesh &m, std::size_t firstN)
        : submeshes(m.submeshes.begin(), m.submeshes.begin()
                    + std::min(firstN, m.submeshes.size()))
        , coverageMask(m.coverageMask)
    {}

    typedef SubMesh::list::iterator iterator;
    typedef SubMesh::list::const_iterator const_iterator;
    iterator begin() { return submeshes.begin(); }
    iterator end() { return submeshes.end(); }
    const_iterator begin() const { return submeshes.begin(); }
    const_iterator end() const { return submeshes.end(); }
    const_iterator cbegin() { return submeshes.begin(); }
    const_iterator cend() { return submeshes.end(); }

    const SubMesh& operator[](std::size_t index) const {
        return submeshes[index];
    }

    SubMesh& operator[](std::size_t index) { return submeshes[index]; }

    /** Adds submesh to this mesh. If mesh is textured (i.e. it has internal
     *  texture coordinates then it is added before first non-textured mesh.
     *
     * \param subMesh submesh to add
     * \return reference to newly added submesh inside this mesh
     */
    SubMesh& add(const SubMesh &subMesh);

    std::size_t size() const { return submeshes.size(); }

    bool empty() const { return submeshes.empty(); }

    void createCoverage(bool fullyCovered);
};

std::uint32_t extraFlags(const Mesh &mesh);
std::uint32_t extraFlags(const Mesh *mesh);
std::uint32_t extraFlags(const Mesh::pointer &mesh);

math::Extents3 extents(const SubMesh &submesh);
math::Extents3 extents(const Mesh &mesh);

/** Calculates geom-extents.
 *  NB: vertices must be in SDS to work properly.
 */
GeomExtents geomExtents(const SubMesh &submesh);

/** Calculates geom-extents.
 *  NB: vertices must be in SDS to work properly.
 */
GeomExtents geomExtents(const Mesh &mesh);

/** Calculates geom-extents. Convert to SDS system using provided convertor.
 */
GeomExtents geomExtents(const CsConvertor &conv, const SubMesh &submesh);

/** Calculates geom-extents. Convert to SDS system using provided convertor.
 */
GeomExtents geomExtents(const CsConvertor &conv, const Mesh &mesh);

struct SubMeshArea {
    double mesh;
    double internalTexture;
    double externalTexture;

    typedef std::vector<SubMeshArea> list;

    SubMeshArea() : mesh(), internalTexture(), externalTexture() {}
};

/** Returns mesh and texture area for given submesh
 */
SubMeshArea area(const SubMesh &submesh);

/** Returns mesh and texture area for given submesh (only faces with all valid
 *  vertices are taken into account.
 *
 *  Prerequisite: mask.size() == submesh.vertices.size()
 */
SubMeshArea area(const SubMesh &submesh, const VertexMask &mask);

/** Low-level area calculator.
 */
SubMeshArea area(const math::Points3d &vertices, const Faces &faces
                 , const math::Points2d *tc, const Faces *facesTc
                 , const math::Points2d *etc
                 , const VertexMask *mask = nullptr);

/** Returns mesh and texture area for given mesh.
 */
struct MeshArea {
    double mesh;
    SubMeshArea::list submeshes;

    MeshArea() : mesh() {}
};

/** Area of whole mesh.
 */
MeshArea area(const Mesh &mesh);

/** Area of whole mesh, with masks.
 */
MeshArea area(const Mesh &mesh, const VertexMasks &masks);

/** Generates external texture coordinates from vertices. Submesh must be in
 *  spatial division SRS.
 *
 *  If external texture is not allowed any existing etc are removed.
 *
 * \param sm submesh to update (must be in SDS)
 * \param sdsExtents extents of tile in SDS
 * \param allowed flags whether exteranl texture is allowed for this submesh
 */
void generateEtc(SubMesh &sm, const math::Extents2 &sdsExtents
                 , bool allowed = true);

/** Generates external texture coordinates from vertices. Whole mesh must be in
 *  spatial division SRS.
 *
 *  If external texture is not allowed any existing etc are removed.
 *
 * \param mesh mesh to update (must be in SDS)
 * \param sdsExtents extents of tile in SDS
 * \param allowed flags whether exteranl texture is allowed for this submesh
 */
void generateEtc(Mesh &mesh, const math::Extents2 &sdsExtents
                 , bool allowed = true);

/** Updates coverage mask by rendering given submesh (must be in spatial
 *  division SRS).
 *
 * \param mesh coverage mask of this mesh is updated
 * \param sm submesh to render (must be in SDS)
 * \param sdsExtents extents of tile in SDS
 * \param smIndex submesh index
 */
void updateCoverage(Mesh &mesh, const SubMesh &sm
                    , const math::Extents2 &sdsExtents
                    , std::uint8_t smIndex = 1);

/** Generate coverage mask by rendering all submeshes (must be in spatial
 *  division SRS).
 *
 * \param mesh coverage mask of this mesh is updated
 * \param sdsExtents extents of tile in SDS
 */
void generateCoverage(Mesh &mesh, const math::Extents2 &sdsExtents);

// IO
void saveMesh(std::ostream &out, const Mesh &mesh
              , const Atlas *atlas = nullptr);
void saveMesh(const boost::filesystem::path &path, const Mesh &mesh
              , const Atlas *atlas = nullptr);

Mesh loadMesh(std::istream &in, const boost::filesystem::path &path
              = "unknown");
Mesh loadMesh(const boost::filesystem::path &path);

void saveMesh(const storage::OStream::pointer &out, const Mesh &mesh
              , const Atlas *atlas = nullptr);
Mesh loadMesh(const storage::IStream::pointer &in);

/** Saves mesh as is.
 */
void saveMeshProper(std::ostream &out, const Mesh &mesh
                    , const Atlas *atlas = nullptr);

multifile::Table readMeshTable(std::istream &is
                               , const boost::filesystem::path &path
                               = "unknown");

struct MeshMask {
    Mesh::CoverageMask coverageMask;
    std::vector<SubMesh::SurfaceReference> surfaceReferences;
};

MeshMask loadMeshMask(std::istream &is
                      , const boost::filesystem::path &path
                      = "unknown");
MeshMask loadMeshMask(const boost::filesystem::path &path);
MeshMask loadMeshMask(const storage::IStream::pointer &in);

// inlines

inline std::uint32_t extraFlags(const Mesh *mesh) {
    return mesh ? extraFlags(*mesh) : 0;
}

inline std::uint32_t extraFlags(const Mesh::pointer &mesh) {
    return extraFlags(mesh.get());
}

inline Mesh::Mesh(bool fullyCovered)
    : coverageMask(coverageOrder, fullyCovered)
{}

inline void Mesh::createCoverage(bool fullyCovered)
{
    coverageMask.recreate(coverageOrder, fullyCovered);
}

inline void SubMesh::cloneMetadataInto(SubMesh &dst) const
{
    dst.textureMode = textureMode;
    dst.textureLayer = textureLayer;
    dst.uvAreaScale = uvAreaScale;
}

UTILITY_GENERATE_ENUM_IO(SubMesh::TextureMode,
    ((internal))
    ((external))
)

inline void saveMesh(const storage::OStream::pointer &out, const Mesh &mesh
                     , const Atlas *atlas)
{
    return saveMesh(*out, mesh, atlas);
}

inline Mesh loadMesh(const storage::IStream::pointer &in)
{
    return loadMesh(*in, in->name());
}

inline MeshMask loadMeshMask(const storage::IStream::pointer &in)
{
    return loadMeshMask(*in, in->name());
}

} } // namespace vadstena::vts

#endif // vadstena_libs_vts_mesh_hpp
