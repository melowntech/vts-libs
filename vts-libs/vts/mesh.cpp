#include "dbglog/dbglog.hpp"

#include "utility/binaryio.hpp"

#include "math/math.hpp"
#include "math/geometry.hpp"
#include "math/transform.hpp"

#include "imgproc/scanconversion.hpp"

#include "half/half.hpp"

#include "../storage/error.hpp"

#include "./mesh.hpp"
#include "./multifile.hpp"

namespace fs = boost::filesystem;
namespace bin = utility::binaryio;

namespace half = half_float::detail;

namespace vadstena { namespace vts {

namespace {
    const char MAGIC[2] = { 'M', 'E' };
    const std::uint16_t VERSION = 1;

    const std::string MF_MAGIC("ME");
    const std::uint16_t MF_VERSION = 1;

    struct SubMeshFlag { enum : std::uint8_t {
        internalTexture = 0x1
        , externalTexture = 0x2
        /*, reserved = 0x4 */
        , textureMode = 0x8
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
    return std::abs
        (math::crossProduct(math::Point2(b - a), math::Point2(c - a)))
        / 2.0;
}

}

SubMeshArea area(const SubMesh &sm)
{
    if (sm.faces.empty()) { return {}; }

    SubMeshArea a;

    // calculate the total area of the faces
    for (const auto &face : sm.faces) {
        a.mesh += detail::triangleArea(sm.vertices[face[0]]
                                       , sm.vertices[face[1]]
                                       , sm.vertices[face[2]]);
    }

    // internal texture
    if (!sm.tc.empty()) {
        for (const auto &face : sm.facesTc) {
            a.internalTexture += detail::triangleArea(sm.tc[face[0]]
                                                      , sm.tc[face[1]]
                                                      , sm.tc[face[2]]);
        }
    }

    // external texture
    if (!sm.etc.empty()) {
        for (const auto &face : sm.faces) {
            a.externalTexture += detail::triangleArea(sm.etc[face[0]]
                                                      , sm.etc[face[1]]
                                                      , sm.etc[face[2]]);
        }
    }

    // NB: internal UV area is multiplied by uvAreaFactor
    a.internalTexture *= sm.uvAreaScale;
    // NB: external texture is not scaled because there are always finer data
    //     for external texture

    return a;
}

MeshArea area(const Mesh &mesh)
{
    MeshArea out;
    for (const auto &sm : mesh) {
        out.submeshes.push_back(area(sm));
        out.mesh += out.submeshes.back().mesh;
    }
    return out;
}

namespace {

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

void saveMeshProperties(std::uint16_t version, std::ostream &out
                        , const Mesh &mesh)
{
    (void) version;

    for (const auto &sm : mesh.submeshes) {
        bin::write(out, double(sm.uvAreaScale));
    }
}

} // namespace

void saveMesh(std::ostream &out, const Mesh &mesh)
{
    multifile::Table table(MF_VERSION, MF_MAGIC);

    auto p(out.tellp());
    saveMeshProper(out, mesh);
    p = table.add(p, out.tellp() - p);

    mesh.coverageMask.dump(out);
    p = table.add(p, out.tellp() - p);

    saveMeshProperties(table.version, out, mesh);
    table.entries.emplace_back(p, out.tellp() - p);

    multifile::writeTable(table, out);
}

void saveMesh(const fs::path &path, const Mesh &mesh)
{
    utility::ofstreambuf f(path.string());
    saveMesh(f, mesh);
    f.close();
}

namespace {

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

void loadMeshProperties(std::uint16_t version, std::istream &in, Mesh &mesh)
{
    (void) version;

    for (auto &sm : mesh.submeshes) {
        double uvAreaScale;
        bin::read(in, uvAreaScale);
        sm.uvAreaScale = uvAreaScale;
    }
}

} // namespace

multifile::Table readMeshTable(std::istream &is
                               , const boost::filesystem::path &path)

{
    return multifile::readTable(is, MF_MAGIC, path)
        .versionAtMost(MF_VERSION, path)
        .checkEntryCount(3, path);
}

Mesh loadMesh(std::istream &in, const fs::path &path)
{
    const auto table(readMeshTable(in, path));

    Mesh mesh;

    in.seekg(table.entries[0].start);
    loadMeshProper(in, path, mesh);

    in.seekg(table.entries[1].start);
    mesh.coverageMask.load(in);

    in.seekg(table.entries[2].start);
    loadMeshProperties(table.version, in, mesh);

    return mesh;
}

Mesh loadMesh(const fs::path &path)
{
    utility::ifstreambuf f(path.string());
    auto mesh(loadMesh(f, path));
    f.close();
    return mesh;
}

SubMesh& Mesh::add(const SubMesh &subMesh)
{
    bool simpleAdd(false);
    if (subMesh.tc.empty()) {
        // new submesh is texture-less
        simpleAdd = true;
    } else if (submeshes.empty()) {
        // empty mesh
        simpleAdd = true;
    } else if (!submeshes.back().tc.empty()) {
        // last submesh is textured
        simpleAdd = true;
    }

    if (simpleAdd) {
        submeshes.push_back(subMesh);
        return submeshes.back();
    }

    // ok, we have to insert new submesh before first non-textured existing
    // submesh

    auto isubmeshes(std::find_if(submeshes.begin(), submeshes.end()
                                 , [](const SubMesh &sm)
    {
        return sm.tc.empty();
    }));

    isubmeshes = submeshes.insert(isubmeshes, subMesh);
    return *isubmeshes;
}

class TextureNormalizer {
public:
    TextureNormalizer(const math::Extents2 &divisionExtents)
        : size_(size(divisionExtents))
        , origin_(divisionExtents.ll)
    {}

    math::Point2 operator()(const math::Point3 &p) const {
        // NB: origin is in the upper-left corner
        return { (p(0) - origin_(0)) / size_.width
                , (p(1) - origin_(1)) / size_.height };
    };

private:
    math::Size2f size_;
    math::Point2 origin_;
};

void generateEtc(SubMesh &sm, const math::Extents2 &sdsExtents, bool allowed)
{
    sm.etc.clear();
    if (!allowed) { return; }

    TextureNormalizer tn(sdsExtents);

    // generate from vertices
    for (const auto &v : sm.vertices) {
        sm.etc.push_back(tn(v));
    }
}

namespace {

/** Geo coordinates to coverage mask mapping.
 * NB: result is in pixel system: pixel centers have integral indices
 */
math::Matrix4 geo2mask(const math::Extents2 &extents
                              , const math::Size2 &gridSize)
{
    math::Matrix4 trafo(boost::numeric::ublas::identity_matrix<double>(4));

    auto es(size(extents));

    // scales
    math::Size2f scale(gridSize.width / es.width
                       , gridSize.height / es.height);

    // scale to grid
    trafo(0, 0) = scale.width;
    trafo(1, 1) = -scale.height;

    // move to origin and also move pixel centers to integral indices
    trafo(0, 3) = -extents.ll(0) * scale.width - 0.5;
    trafo(1, 3) = extents.ur(1) * scale.height - 0.5;

    return trafo;
}

} // namespace

void updateCoverage(Mesh &mesh, const SubMesh &sm
                    , const math::Extents2 &sdsExtents)
{
    auto &cm(mesh.coverageMask);
    const auto rasterSize(cm.size());
    auto trafo(geo2mask(sdsExtents, rasterSize));

    std::vector<imgproc::Scanline> scanlines;
    cv::Point3f tri[3];
    for (const auto &face : sm.faces) {
        for (int i : { 0, 1, 2 }) {
            auto p(transform(trafo, sm.vertices[face[i]]));
            tri[i].x = p(0); tri[i].y = p(1); tri[i].z = p(2);
        }

        scanlines.clear();
        imgproc::scanConvertTriangle(tri, 0, rasterSize.height, scanlines);

        for (const auto &sl : scanlines) {
            imgproc::processScanline
                (sl, 0, rasterSize.width, [&](int x, int y, float)
            {
                cm.set(x, y);
            });
        }
    }
}

} } // namespace vadstena::vts
