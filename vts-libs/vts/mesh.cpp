#include <boost/iostreams/filtering_stream.hpp>
#include <boost/iostreams/filter/gzip.hpp>
#include <boost/iostreams/restrict.hpp>

#include "dbglog/dbglog.hpp"

#include "utility/expect.hpp"
#include "utility/binaryio.hpp"

#include "math/math.hpp"
#include "math/geometry.hpp"
#include "math/transform.hpp"

#include "imgproc/scanconversion.hpp"

#include "half/half.hpp"

#include "../storage/error.hpp"

#include "./mesh.hpp"
#include "./meshio.hpp"
#include "./multifile.hpp"
#include "./math.hpp"
#include "./tileindex.hpp"

namespace fs = boost::filesystem;
namespace bio = boost::iostreams;
namespace bin = utility::binaryio;

namespace half = half_float::detail;

namespace vadstena { namespace vts {

namespace {
    // mesh binary file
    const std::string MF_MAGIC("ME");
    const std::uint16_t MF_VERSION = 1;
} // namespace

std::uint32_t extraFlags(const Mesh &mesh) {
    TileIndex::Flag::value_type flags(0);

    if (mesh.coverageMask.full()) {
        flags |= TileIndex::Flag::watertight;
    }

    if (mesh.submeshes.size() > 1) {
        flags |= TileIndex::Flag::multimesh;
    }

    return flags;
}

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

namespace {

template <typename Vertices>
double faceArea(const Vertices &vertices, const Face &face)
{
    return triangleArea(vertices[face[0]]
                        , vertices[face[1]]
                        , vertices[face[2]]);
}

} // namespace

SubMeshArea area(const math::Points3d &vertices
                 , const Faces &faces
                 , const math::Points2d *tc
                 , const Faces *facesTc
                 , const math::Points2d *etc
                 , const VertexMask *mask)
{
    SubMeshArea a;

    if (faces.empty()) { return a; }

    if (mask) {
        // we have mask and texturing info, process
        utility::expect((vertices.size() == mask->size())
                        , "Submesh vertex list size (%d) different "
                        "from vertex mask size (%d)."
                        , faces.size(), mask->size());
    }

    auto valid([&](const Face &face)
    {
        return ((*mask)[face(0)] && (*mask)[face(1)] && (*mask)[face(2)]);
    });

    if (mask && tc && facesTc) {

        // we have to check 3d face validity before computing
        auto ifacesTc(facesTc->begin());
        for (const auto &face : faces) {
            const auto &faceTc(*ifacesTc++);

            if (!valid(face)) { continue; }

            // valid face, compute both areas
            // mesh
            a.mesh += faceArea(vertices, face);

            // texturing mesh
            a.internalTexture += faceArea(*tc, faceTc);
        }
    } else {
        // other cases
        if (mask) {
            for (const auto &face : faces) {
                if (valid(face)) {
                    a.mesh += faceArea(vertices, face);
                }
            }
        } else {
            for (const auto &face : faces) {
                a.mesh += faceArea(vertices, face);
            }
        }

        // internal texture
        if (tc && facesTc) {
            for (const auto &faceTc : *facesTc) {
                a.internalTexture += faceArea(*tc, faceTc);
            }
        }
    }

    // external texture
    if (etc) {
        for (const auto &face : faces) {
            a.externalTexture += faceArea(*etc, face);
        }
    }

    return a;
}

namespace {

template <typename Container>
const Container* nonempty(const Container &c)
{
    return c.empty() ? nullptr : &c;
}

} // namespace

SubMeshArea area(const SubMesh &sm)
{
    auto a(area(sm.vertices, sm.faces, nonempty(sm.tc)
                , nonempty(sm.facesTc), nonempty(sm.etc)));
    // NB: internal UV area is multiplied by uvAreaFactor
    a.internalTexture *= sm.uvAreaScale;
    // NB: external texture is not scaled because there are always finer data
    //     for external texture

    return a;
}

SubMeshArea area(const SubMesh &sm, const VertexMask &mask)
{
    auto a(area(sm.vertices, sm.faces, nonempty(sm.tc)
                , nonempty(sm.facesTc), nonempty(sm.etc), &mask));
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

MeshArea area(const Mesh &mesh, const VertexMasks &masks)
{
    utility::expect((mesh.submeshes.size() == masks.size())
                    , "Number of submeshes (%d) different "
                    "from number of masks (%d)."
                    , mesh.submeshes.size(), masks.size());

    MeshArea out;
    auto imask(masks.begin());
    for (const auto &sm : mesh) {
        out.submeshes.push_back(area(sm, *imask++));
        out.mesh += out.submeshes.back().mesh;
    }
    return out;
}

SubMesh SubMesh::cleanUp() const
{
    SubMesh ret;

    ret.vertices.reserve(vertices.size());
    ret.etc.reserve(etc.size());
    ret.tc.reserve(tc.size());
    ret.faces.reserve(faces.size());
    ret.facesTc.reserve(facesTc.size());

    bool have_etc = !etc.empty();
    bool have_itc = !tc.empty();
    auto itc = facesTc.cbegin();

    std::vector<int> vindex(vertices.size(), -1);
    std::vector<int> tindex(tc.size(), -1);

    // copy faces, skip degenerate ones
    for (const auto &face : faces)
    {
        if (face(0) != face(1) &&
            face(1) != face(2) &&
            face(2) != face(0))
        {
            ret.faces.push_back(face);
            if (have_itc) {
                ret.facesTc.push_back(*itc);
            }

            // copy vertices, renumber indices
            Face &rface(ret.faces.back());
            for (int i = 0; i < 3; i++)
            {
                int &idx(vindex[rface(i)]);
                if (idx < 0) {
                    idx = ret.vertices.size();
                    ret.vertices.push_back(vertices[rface(i)]);
                    if (have_etc) {
                        ret.etc.push_back(etc[rface(i)]);
                    }
                }
                rface(i) = idx;
            }

            if (have_itc) {
                Face &tface(ret.facesTc.back());
                for (int i = 0; i < 3; i++)
                {
                    int &idx(tindex[tface(i)]);
                    if (idx < 0) {
                        idx = ret.tc.size();
                        ret.tc.push_back(tc[tface(i)]);
                    }
                    tface(i) = idx;
                }
            }
        }

        if (have_itc) { itc++; }
    }

    ret.textureMode = textureMode;
    ret.textureLayer = textureLayer;
    ret.surfaceReference = surfaceReference;
    ret.uvAreaScale = uvAreaScale;

    return ret;
}

namespace {

void saveMeshProperties(std::uint16_t version, std::ostream &out
                        , const Mesh &mesh)
{
    (void) version;

    // write uv scale area
    for (const auto &sm : mesh.submeshes) {
        bin::write(out, double(sm.uvAreaScale));
    }
}

void saveSurfaceMapping(std::ostream &out, const Mesh &mesh)
{
    if ((mesh.submeshes.size() == 1)
        && (mesh.submeshes.front().surfaceReference == 1))
    {
        // no need to save anything
        return;
    }

    // save size
    bin::write(out, std::uint8_t(mesh.submeshes.size()));

    // write surface references
    for (const auto &sm : mesh.submeshes) {
        bin::write(out, std::uint8_t(sm.surfaceReference));
    }
}

} // namespace

void saveMesh(std::ostream &out, const Mesh &mesh
              , const Atlas *atlas)
{
    multifile::Table table(MF_VERSION, MF_MAGIC);

    auto p(out.tellp());

    saveMeshProper(out, mesh, atlas);

    p = table.add(p, out.tellp() - p);

    // save mask + surface references (used by 2d interface)
    mesh.coverageMask.save(out);
    saveSurfaceMapping(out, mesh);
    p = table.add(p, out.tellp() - p);

    saveMeshProperties(table.version, out, mesh);
    table.entries.emplace_back(p, out.tellp() - p);

    multifile::writeTable(table, out);
}

void saveMesh(const fs::path &path, const Mesh &mesh, const Atlas *atlas)
{
    utility::ofstreambuf f(path.string());
    saveMesh(f, mesh, atlas);
    f.close();
}

void saveMeshProper(std::ostream &out, const Mesh &mesh
                    , const Atlas *atlas)
{
    // save gzipped (level=9, a bit bigger buffer)
    bio::filtering_ostream gzipped;
    gzipped.push(bio::gzip_compressor(bio::gzip_params(9), 1 << 16));
    gzipped.push(out);
    detail::saveMeshProper(gzipped, mesh, atlas);
    gzipped.flush();
}

namespace {

void loadMeshProperties(std::uint16_t version, std::istream &in, Mesh &mesh)
{
    (void) version;
    for (auto &sm : mesh.submeshes) {
        double uvAreaScale;
        bin::read(in, uvAreaScale);
        sm.uvAreaScale = uvAreaScale;
    }
}

void loadSurfaceMapping(std::uint16_t version, std::istream &in
                        , MeshMask &mask)
{
    (void) version;

    std::uint8_t count;
    bin::read(in, count);

    // write surface references
    while (count--) {
        std::uint8_t sr;
        bin::read(in, sr);
        mask.surfaceReferences.push_back(sr);
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
    if (storage::gzipped(in)) {
        // looks like a gzip
        bio::filtering_istream gzipped;

        // create and add decompressor
        gzipped.push
            (bio::gzip_decompressor(bio::gzip_params().window_bits, 1 << 16));

        // add input restricted to mesh data
        auto rin(bio::restrict(in, table.entries[0].start,
                                   table.entries[0].size));
        gzipped.push(rin);

        gzipped.exceptions(in.exceptions());

        detail::loadMeshProper(gzipped, path, mesh);
    } else {
        // raw file
        detail::loadMeshProper(in, path, mesh);
    }

    in.seekg(table.entries[1].start);
    mesh.coverageMask.load(in, path);

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

MeshMask loadMeshMask(std::istream &in
                      , const boost::filesystem::path &path)
{
    const auto table(readMeshTable(in, path));

    MeshMask mask;

    in.seekg(table.entries[1].start);
    mask.coverageMask.load(in, path);

    // load surface references if available
    if (std::size_t(in.tellg()) < table.entries[1].end()) {
        loadSurfaceMapping(table.version, in, mask);
    }

    return mask;
}

MeshMask loadMeshMask(const boost::filesystem::path &path)
{
    utility::ifstreambuf f(path.string());
    auto mask(loadMeshMask(f, path));
    f.close();
    return mask;
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

void generateEtc(Mesh &mesh, const math::Extents2 &sdsExtents, bool allowed)
{
    if (!allowed) { return; }
    TextureNormalizer tn(sdsExtents);

    for (auto &sm : mesh) {
        sm.etc.clear();
        // generate from vertices
        for (const auto &v : sm.vertices) {
            sm.etc.push_back(tn(v));
        }
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
                    , const math::Extents2 &sdsExtents
                    , std::uint8_t smIndex)
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
                cm.set(x, y, smIndex + 1);
            });
        }
    }
}

void generateCoverage(Mesh &mesh, const math::Extents2 &sdsExtents)
{
    mesh.createCoverage(false);

    std::uint8_t smIndex(0);
    for (const auto &sm : mesh) {
        updateCoverage(mesh, sm, sdsExtents, smIndex++);
    }
}

} } // namespace vadstena::vts
