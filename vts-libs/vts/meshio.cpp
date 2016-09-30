#include "dbglog/dbglog.hpp"

#include "utility/binaryio.hpp"

#include "math/math.hpp"

#include "../storage/error.hpp"

#include "./mesh.hpp"

namespace fs = boost::filesystem;
namespace bin = utility::binaryio;

namespace vadstena { namespace vts { namespace detail {

namespace {
    // mesh proper
    const char MAGIC[2] = { 'M', 'E' };
    const std::uint16_t VERSION = 2;

    struct SubMeshFlag { enum : std::uint8_t {
        internalTexture = 0x1
        , externalTexture = 0x2
        /*, reserved = 0x4 */
        , textureMode = 0x8
    }; };
} // namespace


void saveMeshProper(std::ostream &out, const Mesh &mesh)
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
        v = std::round(math::clamp(v, 0.0, 1.0)
                       * std::numeric_limits<std::uint16_t>::max());
        bin::write(out, std::uint16_t(v));
    });

    // write header
    bin::write(out, MAGIC);
    bin::write(out, VERSION);

    // no mean undulation
    bin::write(out, double(0.0));

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
        if (sm.textureMode == SubMesh::TextureMode::external) {
            flags |= SubMeshFlag::textureMode;
        }
        bin::write(out, flags);

        // surface reference (defaults to 1)
        bin::write(out, std::uint8_t(sm.surfaceReference));

        // write (external) texture layer information
        if (sm.textureLayer) {
            bin::write(out, std::uint16_t(*sm.textureLayer));
        } else {
            // save zero
            bin::write(out, std::uint16_t(0));
        }

        // write extents
        bin::write(out, bbox.ll(0));
        bin::write(out, bbox.ll(1));
        bin::write(out, bbox.ll(2));
        bin::write(out, bbox.ur(0));
        bin::write(out, bbox.ur(1));
        bin::write(out, bbox.ur(2));

        // write sub-mesh data
        bin::write(out, std::uint16_t(sm.vertices.size()));

        auto ietc(sm.etc.begin());
        for (const auto &vertex : sm.vertices) {
            saveVertexComponent(vertex(0), bbox.ll(0), bbsize(0));
            saveVertexComponent(vertex(1), bbox.ll(1), bbsize(1));
            saveVertexComponent(vertex(2), bbox.ll(2), bbsize(2));

            if (flags & SubMeshFlag::externalTexture) {
                saveTexCoord((*ietc)(0));
                saveTexCoord((*ietc)(1));
                ++ietc;
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


void loadMeshProper(std::istream &in, const fs::path &path, Mesh &mesh)
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

    // ignore mean undulation
    double reserved;
    bin::read(in, reserved);

    std::uint16_t subMeshCount;
    bin::read(in, subMeshCount);

    // make room for sub-meshes and load them all
    mesh.submeshes.resize(subMeshCount);
    for (auto &sm : mesh) {
        std::uint8_t flags;
        bin::read(in, flags);

        if (version >= 2) {
            // submesh surface reference was added in version=2
            std::uint8_t surfaceReference;
            bin::read(in, surfaceReference);
            sm.surfaceReference = surfaceReference;
        }

        // load (external) texture layer information
        std::uint16_t u16;
        bin::read(in, u16);

        if (flags & SubMeshFlag::textureMode) {
            sm.textureMode = SubMesh::TextureMode::external;
            // leave textureLayer undefined if zero
            if (u16) {
                sm.textureLayer = u16;
            }
        }

        // load sub-mesh bounding box
        math::Extents3 bbox;
        bin::read(in, bbox.ll(0));
        bin::read(in, bbox.ll(1));
        bin::read(in, bbox.ll(2));
        bin::read(in, bbox.ur(0));
        bin::read(in, bbox.ur(1));
        bin::read(in, bbox.ur(2));

        math::Point3d bbsize(bbox.ur - bbox.ll);

        std::uint16_t vertexCount;
        bin::read(in, vertexCount);
        sm.vertices.resize(vertexCount);

        if (flags & SubMeshFlag::externalTexture) {
            sm.etc.resize(vertexCount);
        }

        // load all vertex components
        auto ietc(sm.etc.begin());
        for (auto &vertex : sm.vertices) {
            vertex(0) = loadVertexComponent(bbox.ll(0), bbsize(0));
            vertex(1) = loadVertexComponent(bbox.ll(1), bbsize(1));
            vertex(2) = loadVertexComponent(bbox.ll(2), bbsize(2));

            if (flags & SubMeshFlag::externalTexture) {
                (*ietc)(0) = loadTexCoord();
                (*ietc)(1) = loadTexCoord();
                ++ietc;
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
}

} } } // namespace vadstena::vts::detail

