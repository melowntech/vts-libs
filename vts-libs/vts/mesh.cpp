#include "dbglog/dbglog.hpp"

#include "utility/binaryio.hpp"

#include "math/geometry.hpp"
#include "half/half.hpp"

#include "../storage/error.hpp"

#include "./mesh.hpp"

namespace fs = boost::filesystem;
namespace bin = utility::binaryio;

namespace half = half_float::detail;

namespace vadstena { namespace vts {

namespace {
    const char MAGIC[2] = { 'M', 'E' };
    const std::uint16_t VERSION = 1;

    struct SubMeshFlag { enum : std::uint8_t {
        internalTexture = 0x1
        , externalTexture = 0x2
        , perVertexUndulation = 0x4
        , referencesExternalTexture = 0x8
    }; };
} // namespace

math::Extents3 extents(const SubMesh &submesh)
{
    return computeExtents(submesh.vertices);
}

math::Extents3 extents(const Mesh &mesh)
{
    math::Extents3 e(math::InvalidExtents{});
    for (const auto &sm : mesh) {
        e = unite(e, extents(sm));
    }
    return e;
}

namespace detail {

double triangleArea(const math::Point3 &a, const math::Point3 &b,
                    const math::Point3 &c)
{
    return norm_2(math::crossProduct(b - a, c - a)) / 2.0;
}

double triangleArea(const math::Point2 &a, const math::Point2 &b,
                    const math::Point2 &c)
{
    return math::crossProduct(math::Point2(b - a), math::Point2(c - a)) / 2.0;
}

}

std::pair<double, double> area(const SubMesh &sm)
{
    if (sm.faces.empty()) { return { 0.0, 0.0 }; }

    // calculate the total area of the faces in both the XYZ and UV spaces
    double xyzArea(0);
    for (const auto &face : sm.faces) {
        xyzArea += detail::triangleArea(sm.vertices[face[0]]
                                        , sm.vertices[face[1]]
                                        , sm.vertices[face[2]]);
    }

    double uvArea(0);
    for (const auto &face : sm.facesTc) {
        uvArea += detail::triangleArea(sm.tc[face[0]]
                                       , sm.tc[face[1]]
                                       , sm.tc[face[2]]);
    }

    return { xyzArea, uvArea };
}

MeshArea area(const Mesh &mesh)
{
    MeshArea out;
    for (const auto &sm : mesh) {
        auto a(area(sm));
        out.mesh += a.first;
        out.texture.push_back(a.second);
    }
    return out;
}

void saveMesh(std::ostream &out, const Mesh &mesh)
{
    // helper functions
    auto saveVertexComponent([&out](double v, double o, double s) -> void
    {
        bin::write
            (out, std::uint16_t
             (std::round
              (((v - o) * std::numeric_limits<std::uint16_t>::max()) / s)));
    });

    auto saveTexCoord([&out](double v)
    {
        bin::write
            (out, std::uint16_t
             (std::round(v * std::numeric_limits<std::uint16_t>::max())));
    });

    // write header
    bin::write(out, MAGIC);
    bin::write(out, VERSION);
    bin::write(out, mesh.meanUndulation);
    bin::write(out, std::uint16_t(mesh.submeshes.size()));

    // write submeshes
    for (const auto &sm : mesh) {
        auto bbox(extents(sm));
        math::Point3d bbsize(bbox.ur - bbox.ll);

        // build and write flags
        std::uint8_t flags(0);
        if (!sm.tc.empty()) {
            flags |= SubMeshFlag::internalTexture;
        }
        if (!sm.etc.empty()) {
            flags |= SubMeshFlag::externalTexture;
        }
        if (!sm.vertexUndulation.empty()) {
            flags |= SubMeshFlag::perVertexUndulation;
        }
        if (sm.textureLayer) {
            flags |= SubMeshFlag::referencesExternalTexture;
        }
        bin::write(out, flags);

        // load (external) texture layer information
        if (sm.textureLayer) {
            bin::write(out, *sm.textureLayer);
        }

        // write extents
        bin::write(out, bbox.ll);
        bin::write(out, bbox.ur);

        // write sub-mesh data
        bin::write(out, std::uint16_t(sm.vertices.size()));

        auto ietc(sm.etc.begin());
        auto ivertexUndulation(sm.vertexUndulation.begin());
        for (const auto &vertex : sm.vertices) {
            saveVertexComponent(vertex(0), bbox.ll(0), bbsize(0));
            saveVertexComponent(vertex(1), bbox.ll(1), bbsize(1));
            saveVertexComponent(vertex(2), bbox.ll(2), bbsize(2));

            if (flags & SubMeshFlag::externalTexture) {
                saveTexCoord((*ietc)(0));
                saveTexCoord((*ietc)(1));
                ++ietc;
            }

            if (flags & SubMeshFlag::perVertexUndulation) {
                bin::write(out, half::float2half<std::round_to_nearest>
                           (*ivertexUndulation++));
            }
        }

        // save (internal) texture coordinates
        if (flags & SubMeshFlag::internalTexture) {
            bin::write(out, std::uint16_t(sm.tc.size()));
            for (const auto &tc : sm.tc) {
                saveTexCoord(tc(0));
                saveTexCoord(tc(1));
            }
        }

        // save faces
        bin::write(out, std::uint16_t(sm.faces.size()));
        auto ifacesTc(sm.facesTc.begin());

        for (auto &face : sm.faces) {
            bin::write(out, face(0));
            bin::write(out, face(1));
            bin::write(out, face(2));

            // save (optional) texture coordinate indices
            if (flags & SubMeshFlag::internalTexture) {
                bin::write(out, (*ifacesTc)(0));
                bin::write(out, (*ifacesTc)(1));
                bin::write(out, (*ifacesTc)(2));
                ++ifacesTc;
            }
        }
    }
}

void saveMesh(const fs::path &path, const Mesh &mesh)
{
    utility::ofstreambuf f(path.string());
    saveMesh(f, mesh);
    f.close();
}

Mesh loadMesh(std::istream &in, const fs::path &path)
{
    // helper functions
    auto loadVertexComponent([&in](double o, double s) -> double
    {
        std::uint16_t v;
        bin::read(in, v);
        return o + ((v * s) / std::numeric_limits<std::uint16_t>::max());
    });

    auto loadTexCoord([&in]() -> double
    {
        std::uint16_t v;
        bin::read(in, v);
        return (double(v) / std::numeric_limits<std::uint16_t>::max());
    });

    // Load mesh headers first
    char magic[sizeof(MAGIC)];
    std::uint16_t version;

    bin::read(in, magic);
    bin::read(in, version);

    if (std::memcmp(magic, MAGIC, sizeof(MAGIC))) {
        LOGTHROW(err1, storage::BadFileFormat)
            << "File " << path << " is not a VTS mesh file.";
    }
    if (version > VERSION) {
        LOGTHROW(err1, storage::VersionError)
            << "File " << path
            << " has unsupported version (" << version << ").";
    }

    Mesh mesh;

    bin::read(in, mesh.meanUndulation);

    std::uint16_t subMeshCount;
    bin::read(in, subMeshCount);

    // make room for sub-meshes and load them all
    mesh.submeshes.resize(subMeshCount);
    for (auto &sm : mesh) {
        std::uint8_t flags;
        bin::read(in, flags);

        // load (external) texture layer information
        if (flags & SubMeshFlag::referencesExternalTexture) {
            sm.textureLayer = 0.0;
            bin::read(in, *sm.textureLayer);
        }

        // load sub-mesh bounding box
        math::Extents3 bbox;
        bin::read(in, bbox.ll);
        bin::read(in, bbox.ur);
        math::Point3d bbsize(bbox.ur - bbox.ll);

        std::uint16_t vertexCount;
        bin::read(in, vertexCount);
        sm.vertices.resize(vertexCount);

        if (flags & SubMeshFlag::externalTexture) {
            sm.etc.resize(vertexCount);
        }

        if (flags & SubMeshFlag::perVertexUndulation) {
            sm.vertexUndulation.resize(vertexCount);
        }

        // load all vertex components
        auto ietc(sm.etc.begin());
        auto ivertexUndulation(sm.vertexUndulation.begin());
        for (auto &vertex : sm.vertices) {
            vertex(0) = loadVertexComponent(bbox.ll(0), bbsize(0));
            vertex(1) = loadVertexComponent(bbox.ll(1), bbsize(1));
            vertex(2) = loadVertexComponent(bbox.ll(2), bbsize(2));

            if (flags & SubMeshFlag::externalTexture) {
                (*ietc)(0) = loadTexCoord();
                (*ietc)(1) = loadTexCoord();
                ++ietc;
            }

            if (flags & SubMeshFlag::perVertexUndulation) {
                std::uint16_t v;
                bin::read(in, v);
                *ivertexUndulation++ = half::half2float(v);
            }
        }

        // load (internal) texture coordinates
        if (flags & SubMeshFlag::internalTexture) {
            std::uint16_t tcCount;
            bin::read(in, tcCount);
            sm.tc.resize(tcCount);
            for (auto &tc : sm.tc) {
                tc(0) = loadTexCoord();
                tc(1) = loadTexCoord();
            }
        }

        // load faces
        std::uint16_t faceCount;
        bin::read(in, faceCount);
        sm.faces.resize(faceCount);

        if (flags & SubMeshFlag::internalTexture) {
            sm.facesTc.resize(faceCount);
        }
        auto ifacesTc(sm.facesTc.begin());

        for (auto &face : sm.faces) {
            bin::read(in, face(0));
            bin::read(in, face(1));
            bin::read(in, face(2));

            // load (optional) texture coordinate indices
            if (flags & SubMeshFlag::internalTexture) {
                bin::read(in, (*ifacesTc)(0));
                bin::read(in, (*ifacesTc)(1));
                bin::read(in, (*ifacesTc)(2));
                ++ifacesTc;
            }
        }
    }

    return mesh;
}

Mesh loadMesh(const fs::path &path)
{
    utility::ifstreambuf f(path.string());
    auto mesh(loadMesh(f, path));
    f.close();
    return mesh;
}

} } // namespace vadstena::vts

