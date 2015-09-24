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
typedef std::vector<Point3u16> Points3u16;

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

    /** Per-vertex undulation (empty if none, otherwise must have the same size
     *  as vertices).
     */
    std::vector<double> vertexUndulation;

    /** Indices to face vertices.
     */
    Points3u16 faces;

    /** Indices to internal texture coordinates (empty if none, otherwise must
     *  have the same as faces).
     */
    Points3u16 facesTc;

    /** ID of external texture layer.
     */
    boost::optional<std::uint16_t> textureLayer;

    typedef std::vector<SubMesh> list;
    SubMesh() {}
};

struct Mesh {
    typedef std::shared_ptr<Mesh> pointer;
    typedef imgproc::quadtree::RasterMask CoverageMask;

    double meanUndulation;
    SubMesh::list submeshes;
    CoverageMask coverageMask;

    static math::Size2i coverageSize() {
        return math::Size2i(256, 256);
    };

    Mesh()
        : meanUndulation()
        , coverageMask(coverageSize(), CoverageMask::InitMode::FULL)
    {}

    typedef SubMesh::list::iterator iterator;
    typedef SubMesh::list::const_iterator const_iterator;
    iterator begin() { return submeshes.begin(); }
    iterator end() { return submeshes.end(); }
    const_iterator begin() const { return submeshes.begin(); }
    const_iterator end() const { return submeshes.end(); }
    const_iterator cbegin() { return submeshes.begin(); }
    const_iterator cend() { return submeshes.end(); }
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

/** Returns mesh and texture area for given submesh as pair of double
 *  first = mesh area
 *  second = texture area (normalized)
 */
std::pair<double, double> area(const SubMesh &submesh);

/** Returns mesh and texture area for given mesh.
 */
struct MeshArea {
    double mesh;
    std::vector<double> texture;

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
