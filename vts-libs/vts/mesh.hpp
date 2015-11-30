#ifndef vadstena_libs_vts_mesh_hpp
#define vadstena_libs_vts_mesh_hpp

#include <cstdint>
#include <iosfwd>
#include <vector>
#include <memory>

#include <boost/filesystem/path.hpp>

#include "math/geometry_core.hpp"
#include "imgproc/rastermask/quadtree.hpp"

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

    /** UV area scaling factor to artificially scale UV mesh area, i.e. to make
     *  it look bigger.
     *
     * NB: this value is stored outside mesh served to the client!
     */
    double uvAreaScale;

    typedef std::vector<SubMesh> list;
    SubMesh() : textureMode(TextureMode::internal), uvAreaScale(1.0) {}
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

    static constexpr unsigned int meshIndex() { return 0; }

    Mesh()
        : coverageMask(coverageSize(), CoverageMask::InitMode::FULL)
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

    std::size_t size() const { return submeshes.size(); }
};

inline bool watertight(const Mesh &mesh) { return mesh.coverageMask.full(); }
inline bool watertight(const Mesh *mesh) {
    return mesh ? watertight(*mesh) : false;
}
inline bool watertight(const Mesh::pointer &mesh) {
    return watertight(mesh.get());
}

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

void saveMesh(std::ostream &out, const Mesh &mesh);
void saveMesh(const boost::filesystem::path &path, const Mesh &mesh);

Mesh loadMesh(std::istream &in, const boost::filesystem::path &path
              = "unknown");
Mesh loadMesh(const boost::filesystem::path &path);

multifile::Table readMeshTable(std::istream &is
                               , const boost::filesystem::path &path
                               = "unknown");

} } // namespace vadstena::vts

#endif // vadstena_libs_vts_mesh_hpp
