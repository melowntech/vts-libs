#ifndef vadstena_libs_vts_mesh_hpp
#define vadstena_libs_vts_mesh_hpp

#include <cstdint>
#include <iosfwd>
#include <vector>

#include <boost/filesystem/path.hpp>

#include "math/geometry_core.hpp"

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
    double meanUndulation;
    SubMesh::list submeshes;

    Mesh() : meanUndulation() {}
};

void saveMesh(std::ostream &out, const Mesh &mesh);
void saveMesh(const boost::filesystem::path &path, const Mesh &mesh);

Mesh loadMesh(std::istream &in, const boost::filesystem::path &path
              = "unknown");
Mesh loadMesh(const boost::filesystem::path &path);

} } // namespace vadstena::vts

#endif // vadstena_libs_vts_mesh_hpp
