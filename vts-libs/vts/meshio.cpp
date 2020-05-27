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

#include <vector>
#include <numeric>

#include <boost/iostreams/filtering_stream.hpp>
#include <boost/iostreams/filter/gzip.hpp>

#include "dbglog/dbglog.hpp"

#include "utility/binaryio.hpp"
#include "utility/getenv.hpp"

#include "math/math.hpp"

#include "geometry/forsyth.hpp"
#include "geometry/parse-obj.hpp"

#include "../storage/error.hpp"
#include "../registry/referenceframe.hpp"

#include "mesh.hpp"
#include "atlas.hpp"

namespace fs = boost::filesystem;
namespace bio = boost::iostreams;
namespace bin = utility::binaryio;

namespace vtslibs { namespace vts { namespace detail {

namespace {
    const char *NO_MESH_COMPRESSION(utility::getenv("NO_MESH_COMPRESSION"));

    // mesh proper
    const char MAGIC[2] = { 'M', 'E' };
    const std::uint16_t VERSION = 3;

    // quantization coefficients
    const int GeomQuant = 1024;
    const int TexCoordSubPixelBits = 3;
    const int DefaultTextureSize = 256; // if atlas size unknown

    struct SubMeshFlag { enum : std::uint8_t {
        internalTexture = 0x1
        , externalTexture = 0x2
        /*, reserved = 0x4 */
        , textureMode = 0x8
    }; };
} // namespace

#ifndef VTSLIBS_BROWSER_ONLY

namespace {

/** Calculate Forsyth's cache optimized ordering of faces, vertices and
 *  texcoords. On return, forder, vorder, torder contain a new index for each
 *  face, vertex and texcoord, respectively.
 */
void getMeshOrdering(const SubMesh &submesh,
                     std::vector<int> &forder,
                     std::vector<int> &vorder,
                     std::vector<int> &torder)
{
    auto nfaces = int(submesh.faces.size());
    auto nvertices = int(submesh.vertices.size());
    auto ntexcoords = int(submesh.tc.size());

#if 1
    std::vector<geometry::ForsythVertexIndexType> indices;
    indices.reserve(3*nfaces);
    for (const auto &face : submesh.faces) {
        indices.push_back(face(0));
        indices.push_back(face(1));
        indices.push_back(face(2));
    }

    // get face ordering with Forsyth's algorithm
    forder.resize(nfaces);
    geometry::forsythReorder(forder.data(), indices.data(), nfaces, nvertices);
    // TODO: forsythReorder currently does not take texcoords into account!
#else
    // for testing lack of ordering
    forder.resize(nfaces);
    for (int i = 0; i < nfaces; i++) { forder[i] = i; }
#endif

    vorder.assign(nvertices, -1);
    torder.assign(ntexcoords, -1);

    // face ordering induces vertex and texcoord ordering
    int vnext = 0, tnext = 0;
    for (int i = 0; i < nfaces; i++) {

        const auto &face(submesh.faces[forder[i]]);
        for (int j = 0; j < 3; j++) {
            int &vindex = vorder[face(j)];
            if (vindex < 0) { vindex = vnext++; }
        }

        if (submesh.facesTc.size()) {
            const auto &facetc(submesh.facesTc[forder[i]]);
            for (int j = 0; j < 3; j++) {
                int &tindex = torder[facetc(j)];
                if (tindex < 0) { tindex = tnext++; }
            }
        }
    }
    assert(vnext == nvertices);
    assert(tnext == ntexcoords);
}

/** Helper class to write 16-bit signed/unsigned deltas variably as 1-2 bytes
 *  and collect statistics.
 */
class DeltaWriter
{
public:
    DeltaWriter(std::ostream &out)
        : out_(out), nbytes_(), nsmall_(), nbig_() {}

    void writeWord(unsigned word)
    {
        // TODO: throw exception instead
        assert(word < (1 << 15));
        if (word < 0x80) {
            bin::write(out_, std::uint8_t(word));
            nsmall_++;
            nbytes_++;
        }
        else {
            bin::write(out_, std::uint8_t(0x80 | (word & 0x7f)));
            bin::write(out_, std::uint8_t(word >> 7));
            nbig_++;
            nbytes_ += 2;
        }
    }

    void writeDelta(int value, int &last)
    {
        signed delta = value - last;
        unsigned zigzag = (delta << 1) ^ (delta >> 31);
        writeWord(zigzag);
        last = value;
    }

    int nbytes() const { return nbytes_; }
    int nsmall() const { return nsmall_; }
    int nbig() const { return nbig_; }

private:
    std::ostream &out_;
    int nbytes_, nsmall_, nbig_;
};

void invertIndex(const std::vector<int> &in, std::vector<int> &out)
{
    out.resize(in.size());
    for (unsigned i = 0; i < in.size(); i++) {
        out[in[i]] = i;
    }
}

/** Returns size of submesh texture at given index. If atlas is null or there is
 *  no such texture, default size is returned.
 */
math::Size2 textureSize(const Atlas *atlas, std::size_t submesh
                        , const math::Size2 &dflt)
{
    // fallback to default
    if (!atlas || (submesh >= atlas->size())) { return dflt; }

    return atlas->imageSize(submesh);
}

void saveMeshVersion3(std::ostream &out, const ConstSubMeshRange &submeshes
                      , const Atlas *atlas)
{
    // write header
    bin::write(out, MAGIC);
    bin::write(out, std::uint16_t(3));

    // no mean undulation
    bin::write(out, double(0.0));

    bin::write(out, std::uint16_t(submeshes.size()));

    // write submeshes
    int smIndex(int(submeshes.b) - 1);
    for (const SubMesh &submesh : submeshes)
    {
        smIndex++;

        // remove degenerate faces
        SubMesh sm(submesh.cleanUp());

        // get a good ordering of faces, vertices and texcoords
        std::vector<int> forder, vorder, torder;
        getMeshOrdering(sm, forder, vorder, torder);

        auto bbox(extents(sm));
        math::Point3d bbsize(bbox.ur - bbox.ll);
        math::Point3d center(0.5*(bbox.ll + bbox.ur));
        double scale(1.0 / std::max(bbsize(0), std::max(bbsize(1), bbsize(2))));

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

        DeltaWriter dw(out);

        // write delta coded vertices
        int b1 = dw.nbytes();
        {
            // TODO: check: nv must be < 2^16
            auto nv(int(sm.vertices.size()));
            bin::write(out, std::uint16_t(nv));
            bin::write(out, std::uint16_t(GeomQuant));

            std::vector<int> ivorder;
            invertIndex(vorder, ivorder);

            int last[3] = {0, 0, 0};
            for (int i = 0; i < nv; i++) {
                const auto &v(sm.vertices[ivorder[i]]);
                for (int j = 0; j < 3; j++)
                {
                    double ncoord = (v(j) - center(j)) * scale;
                    auto qcoord(int(round(ncoord * GeomQuant)));
                    dw.writeDelta(qcoord, last[j]);
                }
            }

            // write delta coded external coordinates
            if (flags & SubMeshFlag::externalTexture)
            {
                int quant = (int(vtslibs::registry::BoundLayer::tileWidth)
                             << TexCoordSubPixelBits);
                bin::write(out, std::uint16_t(quant));

                int last[2] = {0, 0};
                for (int i = 0; i < nv; i++) {
                    const auto &etc(sm.etc[ivorder[i]]);
                    for (int j = 0; j < 2; j++)
                    {
                        auto qcoord(int(round(etc(j) * quant)));
                        dw.writeDelta(qcoord, last[j]);
                    }
                }
            }
        }

        // write delta coded internal texcoords
        int b2 = dw.nbytes();
        if (flags & SubMeshFlag::internalTexture)
        {
            // TODO: check: ntc must be < 2^16
            auto ntc(int(sm.tc.size()));
            bin::write(out, uint16_t(ntc));

            math::Size2 dflt{DefaultTextureSize, DefaultTextureSize};
            auto ts(textureSize(atlas, smIndex, dflt));

            int quant[2] = {
                int(ts.width) << TexCoordSubPixelBits,
                int(ts.height) << TexCoordSubPixelBits
            };
            bin::write(out, uint16_t(quant[0]));
            bin::write(out, uint16_t(quant[1]));

            std::vector<int> itorder;
            invertIndex(torder, itorder);

            int last[2] = {0, 0};
            for (int i = 0; i < ntc; i++) {
                const auto &t(sm.tc[itorder[i]]);
                for (int j = 0; j < 2; j++)
                {
                    auto qcoord(int(round(t(j) * quant[j])));
                    dw.writeDelta(qcoord, last[j]);
                }
            }
        }

        // write faces
        int b3, b4;
        {
            // TODO: check: nf must be < 2^16
            auto nf(int(sm.faces.size()));
            bin::write(out, uint16_t(nf));

            // write delta coded vertex indices
            b3 = dw.nbytes();
            for (int i = 0, high = -1; i < nf; i++) {
                const auto &face = sm.faces[forder[i]];
                for (int j = 0; j < 3; j++)
                {
                    int index = vorder[face(j)];
                    dw.writeWord(high+1 - index);
                    if (index > high) { high = index; }
                }
            }

            // write delta coded texcoord indices
            b4 = dw.nbytes();
            if (flags & SubMeshFlag::internalTexture)
            {
                for (int i = 0, high = -1; i < nf; i++) {
                    const auto &face = sm.facesTc[forder[i]];
                    for (int j = 0; j < 3; j++)
                    {
                        int index = torder[face(j)];
                        dw.writeWord(high+1 - index);
                        if (index > high) { high = index; }
                    }
                }
            }
        }
#if 0
        int sm = dw.nsmall(), bg = dw.nbig();

        LOG(info1) << "nsmall = " << sm;
        LOG(info1) << "nbig = " << bg << " (" << double(bg)/(bg+sm)*100 << "%).";

        double total = (dw.nbytes() - b1) / 100;
        int vsize = b2 - b1 - 4, tsize = b3 - b2 - 6;
        int isize1 = b4 - b3, isize2 = dw.nbytes() - b4;

        LOG(info1) << "vertices: " << vsize << " B (" << vsize/total << "%).";
        LOG(info1) << "texcoords: " << tsize << " B (" << tsize/total << "%).";
        LOG(info1) << "v. indices: " << isize1 << " B (" << isize1/total << "%).";
        LOG(info1) << "t. indices: " << isize2 << " B (" << isize2/total << "%).";
#endif
        (void) b1; (void) b2; (void) b3; (void) b4;
    }
}

void saveMeshVersion2(std::ostream &out, const ConstSubMeshRange &submeshes)
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
    bin::write(out, std::uint16_t(2));

    // no mean undulation
    bin::write(out, double(0.0));

    bin::write(out, std::uint16_t(submeshes.size()));

    // write submeshes
    for (const auto &sm_ : submeshes) {
        const auto sm(sm_.cleanUp());

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

        // TODO: check faces/facesTc indices to be in range 0-2^16-1
        for (auto &face : sm.faces) {
            bin::write(out, std::uint16_t(face(0)));
            bin::write(out, std::uint16_t(face(1)));
            bin::write(out, std::uint16_t(face(2)));

            // save (optional) texture coordinate indices
            if (flags & SubMeshFlag::internalTexture) {
                bin::write(out, std::uint16_t((*ifacesTc)(0)));
                bin::write(out, std::uint16_t((*ifacesTc)(1)));
                bin::write(out, std::uint16_t((*ifacesTc)(2)));
                ++ifacesTc;
            }
        }
    }
}

} // namespace

void saveMeshProper(std::ostream &out, const ConstSubMeshRange &submeshes
                    , const Atlas *atlas)
{
    // FIXME: this condition is wrong fix when we are sure version3 was not
    // broken since the error was introduced (2017...)
    if (NO_MESH_COMPRESSION) {
        saveMeshVersion3(out, submeshes, atlas);
    } else {
        saveMeshVersion2(out, submeshes);
    }
}

#endif // VTSLIBS_BROWSER_ONLY

////////////////////////////////////////////////////////////////////////////////

namespace {

class DeltaReader
{
public:
    DeltaReader(std::istream &in) : in_(in) {}

    unsigned readWord()
    {
        uint8_t byte1, byte2;
        bin::read(in_, byte1);
        if (byte1 & 0x80) {
            bin::read(in_, byte2);
            return (int(byte1) & 0x7f) | (int(byte2) << 7);
        }
        return byte1;
    }

    int readDelta(int &last)
    {
        unsigned word = readWord();
        // FIXME: fix for proper bit operations
        int delta((word >> 1) ^ (-(word & 1)));
        return (last += delta);
    }

private:
    std::istream &in_;
};


void loadSubmeshVersion3(std::istream &in, SubMesh &sm, std::uint8_t flags
                         , const math::Extents3 &bbox)
{
    math::Point3d center = 0.5*(bbox.ll + bbox.ur);
    math::Point3d bbsize(bbox.ur - bbox.ll);
    double scale = std::max(bbsize(0), std::max(bbsize(1), bbsize(2)));

    DeltaReader dr(in);

    // load vertices
    std::uint16_t vertexCount;
    {
        bin::read(in, vertexCount);
        sm.vertices.resize(vertexCount);

        std::uint16_t quant;
        bin::read(in, quant);

        int last[3] = {0, 0, 0};
        double multiplier = 1.0 / quant;
        for (auto& vertex : sm.vertices) {
            for (int i = 0; i < 3; i++)
            {
                int qcoord = dr.readDelta(last[i]);
                double coord = double(qcoord) * multiplier;
                vertex(i) = coord*scale + center(i);
            }
        }
    }

    // load external coords
    if (flags & SubMeshFlag::externalTexture)
    {
        sm.etc.resize(vertexCount);

        std::uint16_t quant;
        bin::read(in, quant);

        int last[2] = {0, 0};
        double multiplier = 1.0 / quant;
        for (auto& etc : sm.etc) {
            for (int i = 0; i < 2; i++)
            {
                int qcoord = dr.readDelta(last[i]);
                etc(i) = double(qcoord) * multiplier;
            }
        }
    }

    // load texcoords
    if (flags & SubMeshFlag::internalTexture)
    {
        std::uint16_t tcCount;
        bin::read(in, tcCount);
        sm.tc.resize(tcCount);

        std::uint16_t tquant[2];
        bin::read(in, tquant[0]);
        bin::read(in, tquant[1]);

        int last[3] = {0, 0, 0};
        double multiplier[2] = {1.0 / tquant[0], 1.0 / tquant[1]};
        for (auto &texc : sm.tc) {
            for (int i = 0; i < 2; i++)
            {
                int qcoord = dr.readDelta(last[i]);
                texc(i) = double(qcoord) * multiplier[i];
            }
        }
    }

    // load faces
    {
        std::uint16_t faceCount;
        bin::read(in, faceCount);
        sm.faces.resize(faceCount);

        int high = 0;
        for (auto &face : sm.faces) {
            for (int i = 0; i < 3; i++)
            {
                int delta = dr.readWord();
                int index = high - delta;
                if (!delta) { high++; }
                face(i) = index;
            }
        }

        if (flags & SubMeshFlag::internalTexture) {
            sm.facesTc.resize(faceCount);

            int high = 0;
            for (auto &face : sm.facesTc) {
                for (int i = 0; i < 3; i++)
                {
                    int delta = dr.readWord();
                    int index = high - delta;
                    if (!delta) { high++; }
                    face(i) = index;
                }
            }
        }
    }
}

void loadSubmeshVersion2(std::istream &in, SubMesh &sm, std::uint8_t flags
                         , const math::Extents3 &bbox)
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
        std::uint16_t index;
        bin::read(in, index); face(0) = index;
        bin::read(in, index); face(1) = index;
        bin::read(in, index); face(2) = index;

        // load (optional) texture coordinate indices
        if (flags & SubMeshFlag::internalTexture) {
            bin::read(in, index); (*ifacesTc)(0) = index;
            bin::read(in, index); (*ifacesTc)(1) = index;
            bin::read(in, index); (*ifacesTc)(2) = index;
            ++ifacesTc;
        }
    }
}

// get submesh from submesh -> identity
inline SubMesh& getSubmesh(SubMesh& sm) { return sm; }

// get submesh from normalized submesh
inline SubMesh& getSubmesh(NormalizedSubMesh &sm) { return sm.submesh; }

// helpers normalized bbox
const math::Extents3 normBbox(-1.0, -1.0, -1.0, +1.0, +1.0, +1.0);

inline void loadSubmeshVersion2(std::istream &in, NormalizedSubMesh &sm
                                , std::uint8_t flags
                                , const math::Extents3 &bbox)
{
    loadSubmeshVersion2(in, sm.submesh, flags, normBbox);
    sm.extents = bbox;
}

inline void loadSubmeshVersion3(std::istream &in, NormalizedSubMesh &sm
                                , std::uint8_t flags
                                , const math::Extents3 &bbox)
{
    loadSubmeshVersion3(in, sm.submesh, flags, bbox);

    // re-compute extents
    sm.extents = math::computeExtents(sm.submesh.vertices);
    const auto es(math::size(sm.extents));
    const auto center(math::center(sm.extents));

    const math::Point3 scale(2.0 / es.width, 2.0 / es.height, 2.0 / es.depth);

    // normalize
    for (auto &v : sm.submesh.vertices) {
        v(0) = (v(0) - center(0)) * scale(0);
        v(1) = (v(1) - center(1)) * scale(1);
        v(2) = (v(2) - center(2)) * scale(2);
    }
}

template <typename MeshType>
void loadMeshProperImpl(std::istream &in, const fs::path &path
                        , MeshType &mesh)
{
    // Load mesh headers first
    char magic[sizeof(MAGIC)];
    std::uint16_t version;

    bin::read(in, magic);
    bin::read(in, version);

    LOG(info1) << "Mesh version: " << version;

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
    mesh.resize(subMeshCount);
    for (auto &meshItem : mesh) {
        auto &sm(getSubmesh(meshItem));

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

        if (version >= 3) {
            loadSubmeshVersion3(in, meshItem, flags, bbox);
        } else {
            loadSubmeshVersion2(in, meshItem, flags, bbox);
        }
    }
}

} // namespace

void loadMeshProper(std::istream &in, const boost::filesystem::path &path
                    , Mesh &mesh)
{
    loadMeshProperImpl(in, path, mesh.submeshes);
}

} // namespace detail

NormalizedSubMesh::list
loadMeshProperNormalized(std::istream &in
                         , const boost::filesystem::path &path)
{
    NormalizedSubMesh::list submeshes;

    if (storage::gzipped(in)) {
        // looks like a gzip
        bio::filtering_istream gzipped;

        // create and add decompressor
        gzipped.push
            (bio::gzip_decompressor(bio::gzip_params().window_bits, 1 << 16));

        // add input restricted to mesh data
        gzipped.push(in);
        gzipped.exceptions(in.exceptions());

        detail::loadMeshProperImpl(gzipped, path, submeshes);
    } else {
        // raw file
        detail::loadMeshProperImpl(in, path, submeshes);
    }

    return submeshes;
}

void saveSubMeshAsObj(std::ostream &out, const SubMesh &sm
                      , std::size_t index, const Atlas*
                      , const std::string &matlib)
{
    out.setf(std::ios::scientific, std::ios::floatfield);

    const bool hasTc(!sm.facesTc.empty());

    if (hasTc && !matlib.empty()) {
        out << "mtllib " << matlib << '\n';
    }

    for (const auto &vertex : sm.vertices) {
        out << "v " << vertex(0) << ' ' << vertex(1) << ' '  << vertex(2)
            << '\n';
    }

    if (hasTc) {
        for (const auto &tc : sm.tc) {
            out << "vt " << tc(0) << ' ' << tc(1) << '\n';
        }
    }

    if (hasTc && !matlib.empty()) {
        out << "usemtl " << index << '\n';
    }

    auto ifacesTc(sm.facesTc.begin());
    for (const auto &face : sm.faces) {
        if (hasTc) {
            const auto &faceTc(*ifacesTc++);
            out << "f " << (face(0) + 1) << '/' << (faceTc(0) + 1)
                << ' ' << (face(1) + 1) << '/' << (faceTc(1) + 1)
                << ' ' << (face(2) + 1) << '/' << (faceTc(2) + 1)
                << '\n';
        } else {
            out << "f " << (face(0) + 1) << ' ' << (face(1) + 1) << ' '
                << (face(2) + 1) << '\n';
        }
    }
}

void saveSubMeshAsObj(const boost::filesystem::path &filepath
                      , const SubMesh &sm, std::size_t index
                      , const Atlas *atlas
                      , const std::string &matlib)
{
    LOG(info2) << "Saving submesh to file <" << filepath << ">.";

    std::ofstream f;
    f.exceptions(std::ios::badbit | std::ios::failbit);
    try {
        f.open(filepath.string(), std::ios_base::out | std::ios_base::trunc);
    } catch (const std::exception&) {
        LOGTHROW(err3, std::runtime_error)
            << "Unable to save mesh to <" << filepath << ">.";
    }
    saveSubMeshAsObj(f, sm, index, atlas, matlib);
}

namespace {

class ObjLoader : public geometry::ObjParserBase {
public:
    ObjLoader()
        : textureId_(0), vMap_(), tcMap_()
    {
        // make sure we have at least one valid material
        useMaterial(0);
    }

    /** Steal mesh.
     */
    vts::Mesh&& mesh() { return std::move(mesh_); }

private:
    typedef std::vector<int> VertexMap;
    typedef std::vector<VertexMap> VertexMaps;

    virtual void addVertex(const Vector3d &v) {
        vertices_.emplace_back(v.x, v.y, v.z);
    }

    virtual void addTexture(const Vector3d &t) {
        tc_.emplace_back(t.x, t.y);
    }

    template <typename VertexType>
    void addFace(const int f[3], vts::Face &face
                 , const std::vector<VertexType> &vertices
                 , std::vector<VertexType> &out
                 , VertexMap &vmap)
    {
        for (int i(0); i < 3; ++i) {
            const std::size_t src(f[i]);
            // ensure space in map
            if (vmap.size() <= src) { vmap.resize(src + 1, -1); }

            auto &dst(vmap[src]);
            if (dst < 0) {
                // new mapping
                dst = int(out.size());
                out.push_back(vertices[src]);
            }
            face(i) = dst;
        }
    }

    virtual void addFacet(const Facet &f) {
        auto &sm(mesh_.submeshes[textureId_]);
        sm.faces.emplace_back();
        addFace(f.v, sm.faces.back(), vertices_, sm.vertices, *vMap_);

        sm.facesTc.emplace_back();
        addFace(f.t, sm.facesTc.back(), tc_, sm.tc, *tcMap_);
    }

    virtual void useMaterial(const std::string &m) {
        // get new material index
        useMaterial(boost::lexical_cast<unsigned int>(m));
    }

    void useMaterial(unsigned int textureId) {
        textureId_ = textureId;

        // ensure space in all lists
        if (mesh_.submeshes.size() <= textureId_) {
            mesh_.submeshes.resize(textureId_ + 1);
            vMaps_.resize(textureId_ + 1);
            tcMaps_.resize(textureId_ + 1);

            vMap_ = &vMaps_[textureId_];
            tcMap_ = &tcMaps_[textureId_];
        }
    }

    virtual void addNormal(const Vector3d&) { /*ignored*/ }
    virtual void materialLibrary(const std::string&) { /*ignored*/ }

    math::Points3 vertices_;
    math::Points2 tc_;
    VertexMaps vMaps_;
    VertexMaps tcMaps_;

    vts::Mesh mesh_;
    unsigned int textureId_;

    VertexMap *vMap_;
    VertexMap *tcMap_;
};

} // namespace

Mesh loadMeshFromObj(std::istream &is
                     , const boost::filesystem::path &path)
{
    ObjLoader loader;
    if (!loader.parse(is)) {
        LOGTHROW(err2, std::runtime_error)
            << "Unable to load mesh from OBJ file at " << path << ".";
    }
    return loader.mesh();
}

} } // namespace vadstena::vts
