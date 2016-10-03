#include "dbglog/dbglog.hpp"

#include "utility/binaryio.hpp"

#include "math/math.hpp"

#include "../storage/error.hpp"
#include "../registry/referenceframe.hpp"

#include "./mesh.hpp"

namespace fs = boost::filesystem;
namespace bin = utility::binaryio;

namespace vadstena { namespace vts { namespace detail {

namespace {
    // mesh proper
    const char MAGIC[2] = { 'M', 'E' };
    const std::uint16_t VERSION = 3;

    // quantization coefficients
    const int GeomQuant = 1024;
    const int TexCoordQuant = 2048; // FIXME
    const int SubPixelBits = 2;

    struct SubMeshFlag { enum : std::uint8_t {
        internalTexture = 0x1
        , externalTexture = 0x2
        /*, reserved = 0x4 */
        , textureMode = 0x8
    }; };
} // namespace

namespace {

#define FORSYTH_IMPLEMENTATION
#include "./forsyth.h"

/** Calculate Forsyth's cache optimized ordering of faces, vertices and
 *  texcoords. On return, forder, vorder, torder contain a new index for each
 *  face, vertex and texcoord, respectively.
 */
void getMeshOrdering(const SubMesh &submesh,
                     std::vector<int> &forder,
                     std::vector<int> &vorder,
                     std::vector<int> &torder)
{
    int nfaces = submesh.faces.size();
    int nvertices = submesh.vertices.size();
    int ntexcoords = submesh.tc.size();

#if 1
    std::vector<ForsythVertexIndexType> indices;
    indices.reserve(3*nfaces);
    for (const auto &face : submesh.faces) {
        indices.push_back(face(0));
        indices.push_back(face(1));
        indices.push_back(face(2));
    }

    // get face ordering with Forsyth's algorithm
    forder.resize(nfaces);
    forsythReorder(forder.data(), indices.data(), nfaces, nvertices);
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

void saveMeshVersion3(std::ostream &out, const Mesh &mesh)
{
    // write header
    bin::write(out, MAGIC);
    bin::write(out, std::uint16_t(3));

    // no mean undulation
    bin::write(out, double(0.0));

    bin::write(out, std::uint16_t(mesh.submeshes.size()));

    // write submeshes
    for (const SubMesh &sm : mesh)
    {
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
            int nv(sm.vertices.size());
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
                    int qcoord = round(ncoord * GeomQuant);
                    dw.writeDelta(qcoord, last[j]);
                }
            }

            // write delta coded external coordinates
            if (flags & SubMeshFlag::externalTexture)
            {
                const auto &tsize(vadstena::registry::BoundLayer::tileSize());
                int quant = tsize.width * (1 << SubPixelBits);
                bin::write(out, std::uint16_t(quant));

                int last[2] = {0, 0};
                for (int i = 0; i < nv; i++) {
                    const auto &etc(sm.etc[ivorder[i]]);
                    for (int j = 0; j < 2; j++)
                    {
                        int qcoord = round(etc(j) * quant);
                        dw.writeDelta(qcoord, last[j]);
                    }
                }
            }
        }

        // write delta coded internal texcoords
        int b2 = dw.nbytes();
        if (flags & SubMeshFlag::internalTexture)
        {
            int ntc(sm.tc.size());
            bin::write(out, uint16_t(ntc));
            bin::write(out, uint16_t(TexCoordQuant)); // FIXME
            bin::write(out, uint16_t(TexCoordQuant));

            std::vector<int> itorder;
            invertIndex(torder, itorder);

            int last[2] = {0, 0};
            for (int i = 0; i < ntc; i++) {
                const auto &t(sm.tc[itorder[i]]);
                for (int j = 0; j < 2; j++)
                {
                    int qcoord = round(t(j) * TexCoordQuant);
                    dw.writeDelta(qcoord, last[j]);
                }
            }
        }

        // write faces
        int b3, b4;
        {
            int nf(sm.faces.size());
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

void saveMeshVersion2(std::ostream &out, const Mesh &mesh)
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

} // namespace

void saveMeshProper(std::ostream &out, const Mesh &mesh)
{
/*    if (std::getenv("USE_MESH_COMPRESSION")) {
        saveMeshVersion3(out, mesh);
    }
    else*/ {
        saveMeshVersion2(out, mesh);
    }
}


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
        int delta = (word >> 1) ^ (-(word & 1));
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

} // namespace

void loadMeshProper(std::istream &in, const fs::path &path, Mesh &mesh)
{
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

        if (version >= 3) {
            loadSubmeshVersion3(in, sm, flags, bbox);
        }
        else {
            loadSubmeshVersion2(in, sm, flags, bbox);
        }
    }
}

} } } // namespace vadstena::vts::detail
