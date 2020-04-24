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
#include <boost/iostreams/filtering_stream.hpp>
#include <boost/iostreams/filter/gzip.hpp>
#include <boost/iostreams/restrict.hpp>

#include "dbglog/dbglog.hpp"

#include "utility/expect.hpp"
#include "utility/binaryio.hpp"

#include "math/math.hpp"
#include "math/geometry.hpp"
#include "math/transform.hpp"

#include "half/half.hpp"

#include "../storage/error.hpp"

#include "mesh.hpp"
#include "meshio.hpp"
#include "multifile.hpp"
#include "math.hpp"
#include "tileindex.hpp"

namespace fs = boost::filesystem;
namespace bio = boost::iostreams;
namespace bin = utility::binaryio;

namespace half = half_float::detail;

namespace vtslibs { namespace vts {

namespace {
    // mesh binary file
    const std::string MF_MAGIC("ME");

    const std::uint16_t MF_VERSION_OLD = 1;
    const std::uint16_t MF_VERSION_PROPERTY_FLAGS = 2;

    const std::uint16_t MF_VERSION = MF_VERSION_OLD;
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

math::Extents3 extents(const ConstSubMeshRange &smRange)
{
    math::Extents3 e(math::InvalidExtents{});
    for (const auto &sm : smRange) {
        e = unite(e, extents(sm));
    }
    return e;
}

GeomExtents geomExtents(const math::Points3d &vertices)
{
    GeomExtents ge;
    for (const auto &v : vertices) {
        update(ge, v(2));
    }
    return ge;
}

GeomExtents geomExtents(const SubMesh &submesh)
{
    return geomExtents(submesh.vertices);
}

GeomExtents geomExtents(const ConstSubMeshRange &smRange)
{
    GeomExtents ge;
    for (const auto &sm : smRange) {
        update(ge, geomExtents(sm));
    }
    return ge;
}

GeomExtents geomExtents(const CsConvertor &conv, const SubMesh &submesh)
{
    GeomExtents ge;
    for (const auto &v : submesh.vertices) {
        update(ge, conv(v)(2));
    }
    return ge;
}

GeomExtents geomExtents(const CsConvertor &conv
                        , const ConstSubMeshRange &smRange)
{
    GeomExtents ge;
    for (const auto &sm : smRange) {
        update(ge, geomExtents(conv, sm));
    }
    return ge;
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
    cloneMetadataInto(ret);

    // make room
    ret.vertices.reserve(vertices.size());
    ret.etc.reserve(etc.size());
    ret.tc.reserve(tc.size());
    ret.faces.reserve(faces.size());
    ret.facesTc.reserve(facesTc.size());

    bool hasEtc(!etc.empty());
    bool hasTc(!tc.empty());

    typedef std::vector<int> Index;

    struct Mapping {
        int vertex;
        int chain;
        Mapping(int vertex = -1) : vertex(vertex), chain(-1) {}
        typedef std::vector<Mapping> list;
    };

    Index vindex(vertices.size(), -1);
    Index tindex;
    Mapping::list tcMap;

    if (hasTc) {
        // populate only if we have internal textures
        tindex.assign(tc.size(), -1);
        tcMap.reserve(tc.size());
    }

    // Adds 3D mesh vertex to the output mesh, reused already added one
    const auto &addVertex([&](Face::value_type &vertex)
    {
        int &idx(vindex[vertex]);
        if (idx < 0) {
            idx = int(ret.vertices.size());
            ret.vertices.push_back(vertices[vertex]);
            if (hasEtc) { ret.etc.push_back(etc[vertex]); }
        }
        vertex = idx;
    });

    /** Allocates new tc index in the output mesh.
     */
    const auto &allocateTcIndex([&](Face::value_type &tcVertex, int vertex)
    {
        int idx(ret.tc.size());
        ret.tc.push_back(tc[tcVertex]);
        tcMap.emplace_back(vertex);
        return idx;
    });

    /** Adds 2D texturing coordinates to the output mesh, resuses already added
     *  one. Clones texturing coordinates if more than one 3D vertex maps to
     *  tc.
     */
    const auto &addTc([&](Face::value_type &tcVertex, int vertex)
    {
        int &idx(tindex[tcVertex]);

        if (idx < 0) {
            // new tc
            tcVertex = allocateTcIndex(tcVertex, vertex);
            return;
        }

        // already mapped tc, find matching vertex
        auto &mapping(tcMap[idx]);

        while (mapping.vertex != vertex) {
            // different vertex, check
            if (mapping.chain < 0) {
                // end of chain, make new entry
                const auto prev(idx);
                idx = allocateTcIndex(tcVertex, vertex);
                tcMap[prev].chain = idx;
                break;
            }

            // try next one
            idx = mapping.chain;
            mapping = tcMap[idx];
        }

        // map
        tcVertex = idx;
    });

    // copy faces, skip degenerate ones
    auto itc(facesTc.cbegin());
    for (const auto &face : faces) {
        // skip degenerate
        if ((face(0) == face(1)) || (face(1) == face(2))
            || (face(2) == face(0)))
        {
            if (hasTc) { ++itc; }
            continue;
        }

        // copy face, copy vertices, renumber indices
        ret.faces.push_back(face);
        auto &rface(ret.faces.back());
        for (int i = 0; i < 3; i++) { addVertex(rface(i)); }

        if (hasTc) {
            // same for texturing face but resolve different vertices mapping to
            // the same tc
            ret.facesTc.push_back(*itc++);
            auto &tface(ret.facesTc.back());
            for (int i = 0; i < 3; i++) {
                addTc(tface(i), rface(i));
            }
        }
    }

    return ret;
}

namespace {

std::uint8_t MF_PROPERTY_ZINDEX = 1;

/** Compose properties flags.
 */
std::uint8_t propertiesFlags(const Mesh &mesh)
{
    std::uint8_t flags(0);
    for (const auto &sm : mesh.submeshes) {
        if (sm.zIndex) { flags |= MF_PROPERTY_ZINDEX; }
        // other properties check goes here
    }
    return flags;
}

void saveMeshProperties(std::uint16_t version, std::ostream &out
                        , const Mesh &mesh, std::uint8_t flags)
{
    if (flags) { bin::write<std::uint8_t>(out, flags); }

    // write properties
    for (const auto &sm : mesh.submeshes) {
        bin::write(out, double(sm.uvAreaScale));
        if (flags & MF_PROPERTY_ZINDEX) {
            bin::write<std::uint32_t>(out, sm.zIndex);
        }
    }

    (void) version;
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
    const auto flags(propertiesFlags(mesh));
    const auto version(flags ? MF_VERSION_PROPERTY_FLAGS : MF_VERSION_OLD);

    multifile::Table table(version, MF_MAGIC);

    auto p(out.tellp());

    saveMeshProper(out, mesh, atlas);

    p = table.add(p, out.tellp() - p);

    // save mask + surface references (used by 2d interface)
    mesh.coverageMask.save(out);
    saveSurfaceMapping(out, mesh);
    p = table.add(p, out.tellp() - p);

    saveMeshProperties(table.version, out, mesh, flags);
    table.entries.emplace_back(p, out.tellp() - p);

    multifile::writeTable(table, out);
}

void saveMesh(const fs::path &path, const Mesh &mesh, const Atlas *atlas)
{
    utility::ofstreambuf f(path.string());
    saveMesh(f, mesh, atlas);
    f.close();
}

void saveMeshProper(std::ostream &out, const ConstSubMeshRange &submeshes
                    , const Atlas *atlas, bool compress)
{
    if (!compress) {
        // non-compressed
        detail::saveMeshProper(out, submeshes, atlas);
        return;
    }

    // compressed: save gzipped (level=9, a bit bigger buffer)
    bio::filtering_ostream gzipped;
    gzipped.push(bio::gzip_compressor(bio::gzip_params(9), 1 << 16));
    gzipped.push(out);
    detail::saveMeshProper(gzipped, submeshes, atlas);
    gzipped.flush();
}

namespace {

void loadMeshProperties(std::uint16_t version, std::istream &in, Mesh &mesh)
{
    std::uint8_t flags(0);
    if (version >= MF_VERSION_PROPERTY_FLAGS) {
        flags = bin::read<std::uint8_t>(in);
    }

    for (auto &sm : mesh.submeshes) {
        sm.uvAreaScale = bin::read<double>(in);
        if (flags & MF_PROPERTY_ZINDEX) {
            sm.zIndex = bin::read<std::uint32_t>(in);
        }
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

} } // namespace vtslibs::vts
