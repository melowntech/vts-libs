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

#include "./multifile.hpp"

namespace vadstena { namespace vts {

typedef math::Point3_<std::uint16_t> Point3u16;
typedef Point3u16 Face;
typedef std::vector<Face> Faces;

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
};

/**
 */
struct Mesh {
    typedef std::shared_ptr<Mesh> pointer;
    typedef imgproc::quadtree::RasterMask CoverageMask;

    SubMesh::list submeshes;
    CoverageMask coverageMask;

    static math::Size2i coverageSize() {
        return math::Size2i(256, 256);
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
};

bool watertight(const Mesh &mesh);
bool watertight(const Mesh *mesh);
bool watertight(const Mesh::pointer &mesh);
math::Extents3 extents(const SubMesh &submesh);
math::Extents3 extents(const Mesh &mesh);

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

/** Returns mesh and texture area for given mesh.
 */
struct MeshArea {
    double mesh;
    SubMeshArea::list submeshes;

    MeshArea() : mesh() {}
};

MeshArea area(const Mesh &mesh);

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

/** Updates coverage mask by rendering given submeshe (must be in spatial
 *  division SRS).
 *
 * \param mesh coverage mask of this mesh is updated
 * \param sm submesh to render (must be in SDS)
 * \param sdsExtents extents of tile in SDS
 */
void updateCoverage(Mesh &mesh, const SubMesh &sm
                    , const math::Extents2 &sdsExtents);

// IO
void saveMesh(std::ostream &out, const Mesh &mesh);
void saveMesh(const boost::filesystem::path &path, const Mesh &mesh);

Mesh loadMesh(std::istream &in, const boost::filesystem::path &path
              = "unknown");
Mesh loadMesh(const boost::filesystem::path &path);

void saveMesh(const storage::OStream::pointer &out, const Mesh &mesh);
Mesh loadMesh(const storage::IStream::pointer &in);

multifile::Table readMeshTable(std::istream &is
                               , const boost::filesystem::path &path
                               = "unknown");

// inlines

inline bool watertight(const Mesh &mesh) { return mesh.coverageMask.full(); }
inline bool watertight(const Mesh *mesh) {
    return mesh ? watertight(*mesh) : false;
}
inline bool watertight(const Mesh::pointer &mesh) {
    return watertight(mesh.get());
}

inline Mesh::Mesh(bool fullyCovered)
    : coverageMask(coverageSize()
                   , (fullyCovered
                      ? CoverageMask::InitMode::FULL
                      : CoverageMask::InitMode::EMPTY))
{}

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

inline void saveMesh(const storage::OStream::pointer &out, const Mesh &mesh)
{
    return saveMesh(*out, mesh);
}

inline Mesh loadMesh(const storage::IStream::pointer &in)
{
    return loadMesh(*in, in->name());
}

} } // namespace vadstena::vts

#endif // vadstena_libs_vts_mesh_hpp
