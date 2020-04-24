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
#ifndef vtslibs_vts_mesh_hpp
#define vtslibs_vts_mesh_hpp

#include <cstdint>
#include <iosfwd>
#include <vector>
#include <memory>

#include <boost/filesystem/path.hpp>

#include "utility/enum-io.hpp"
#include "math/geometry_core.hpp"

#include "../storage/streams.hpp"

#include "qtree.hpp"
#include "multifile.hpp"
#include "geomextents.hpp"
#include "csconvertor.hpp"

namespace vtslibs { namespace vts {

typedef math::Point3_<std::uint32_t> Point3u32;
typedef Point3u32 Face;
typedef std::vector<Face> Faces;
typedef std::vector<bool> VertexMask;
typedef std::vector<VertexMask> VertexMasks;

// fwd
class Atlas;

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

    typedef std::uint8_t SurfaceReference;

    /** Surface reference. One-based. Defaults to 1.
     */
    SurfaceReference surfaceReference;

    /** UV area scaling factor to artificially scale UV mesh area, i.e. to make
     *  it look bigger.
     *
     * NB: this value is stored outside mesh served to the client!
     */
    double uvAreaScale;

    typedef std::uint32_t ZIndex;

    /** Z index for vertically cut tiles. Tile merging takes this value into
     *  the same way as surface reference.
     *
     * NB: this value is stored outside mesh served to the client!
     */
    ZIndex zIndex;

    typedef std::vector<SubMesh> list;

    SubMesh()
        : textureMode(TextureMode::internal), surfaceReference(1)
        , uvAreaScale(1.0)
        , zIndex()
    {}

    /** Clones metadata (texture mod, layer etc.).
     */
    void cloneMetadataInto(SubMesh &dst) const;

    bool empty() const { return vertices.empty(); }

    /** Filter out degenerate faces and unused vertices, return a new submesh.
     *
     *  Ensures that two distinct vertices do not share the same texture
     *  coordinate. While not enforced by mesh format it makes rendering easier.
     */
    SubMesh cleanUp() const;
};

/** Mesh with submeshes and mask.
 */
struct Mesh {
    typedef std::shared_ptr<Mesh> pointer;
    typedef QTree CoverageMask;

    SubMesh::list submeshes;
    CoverageMask coverageMask;

    static constexpr int coverageOrder = 8;

    static math::Size2i coverageSize() {
        return math::Size2i(1 << coverageOrder, 1 << coverageOrder);
    };

    /** Index inside multifile where mesh is stored.
     */
    static constexpr unsigned int meshIndex() { return 0; }

    /** Creates new (emtpty) mesh.
     *
     *  \param fullyCovered specifies how to initialize coverage mask (true ->
     *  full, false -> empty)
     */
    Mesh(bool fullyCovered = true);

    Mesh(const Mesh&) = default;

    /** Copy first N submeshes from source.
     */
    Mesh(const Mesh &m, std::size_t firstN)
        : submeshes(m.submeshes.begin(), m.submeshes.begin()
                    + std::min(firstN, m.submeshes.size()))
        , coverageMask(m.coverageMask)
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

    /** Adds submesh to this mesh. If mesh is textured (i.e. it has internal
     *  texture coordinates then it is added before first non-textured mesh.
     *
     * \param subMesh submesh to add
     * \return reference to newly added submesh inside this mesh
     */
    SubMesh& add(const SubMesh &subMesh);

    std::size_t size() const { return submeshes.size(); }

    bool empty() const { return submeshes.empty(); }

    void createCoverage(bool fullyCovered);

    /** Resets surface reference to default.
     */
    void resetSurfaceReference() {
        for (auto &sm : submeshes) { sm.surfaceReference = 1; }
    }
};

struct SubMeshRange {
    typedef SubMesh::list::size_type index_type;
    typedef SubMesh::list::iterator iterator;
    typedef SubMesh::list::const_iterator const_iterator;

    std::reference_wrapper<SubMesh::list> submeshes;
    index_type b;
    index_type e;

    SubMeshRange(SubMesh::list &submeshes)
        : submeshes(submeshes), b(0), e(submeshes.size())
    {}

    SubMeshRange(SubMesh::list &submeshes, index_type begin)
        : submeshes(submeshes), b(begin), e(submeshes.size())
    {}

    SubMeshRange(SubMesh::list &submeshes, index_type begin
                 , index_type end)
        : submeshes(submeshes), b(begin), e(end)
    {}

    index_type size() const { return e - b; }
    index_type total() const { return submeshes.get().size(); }
    bool empty() const { return b >= e; }

    iterator begin() {
        return std::next(submeshes.get().begin(), + b);
    }

    iterator end() {
        return std::next(submeshes.get().begin(), + e);
    }

    const_iterator begin() const {
        return std::next(submeshes.get().begin(), + b);
    }

    const_iterator end() const {
        return std::next(submeshes.get().begin(), + e);
    }

    const_iterator cbegin() const {
        return begin();
    }

    const_iterator cend() const {
        return end();
    }
};

struct ConstSubMeshRange {
    typedef SubMesh::list::size_type index_type;
    typedef SubMesh::list::const_iterator const_iterator;

    std::reference_wrapper<const SubMesh::list> submeshes;
    index_type b;
    index_type e;

    ConstSubMeshRange(const SubMesh::list &submeshes)
        : submeshes(submeshes), b(0), e(submeshes.size())
    {}

    ConstSubMeshRange(const SubMesh::list &submeshes, index_type begin)
        : submeshes(submeshes), b(begin), e(submeshes.size())
    {}

    ConstSubMeshRange(const SubMesh::list &submeshes, index_type begin
                      , index_type end)
        : submeshes(submeshes), b(begin), e(end)
    {}

    ConstSubMeshRange(const SubMeshRange &r)
        : submeshes(r.submeshes), b(r.b), e(r.e)
    {}

    index_type size() const { return e - b; }
    index_type total() const { return submeshes.get().size(); }
    bool empty() const { return b >= e; }

    const_iterator begin() const {
        return std::next(submeshes.get().begin(), + b);
    }

    const_iterator end() const {
        return std::next(submeshes.get().begin(), + e);
    }

    const_iterator cbegin() const {
        return begin();
    }

    const_iterator cend() const {
        return end();
    }
};

template <typename ...Args>
SubMeshRange submeshRange(Mesh &mesh, Args &&...args);
template <typename ...Args>
SubMeshRange submeshRange(SubMesh::list &submeshes, Args &&...args);

template <typename ...Args>
ConstSubMeshRange submeshRange(const Mesh &mesh, Args &&...args);
template <typename ...Args>
ConstSubMeshRange submeshRange(const SubMesh::list &submeshes
                               , Args &&...args);

/** Single submesh area
 */
struct SubMeshArea {
    double mesh;
    double internalTexture;
    double externalTexture;

    typedef std::vector<SubMeshArea> list;

    SubMeshArea() : mesh(), internalTexture(), externalTexture() {}
};

/** Whole mesh area
 */
struct MeshArea {
    double mesh;
    SubMeshArea::list submeshes;

    MeshArea() : mesh() {}
};

/** Only mesh mask (with references to surfaces)
 */
struct MeshMask {
    Mesh::CoverageMask coverageMask;
    std::vector<SubMesh::SurfaceReference> surfaceReferences;

    void createCoverage(bool fullyCovered);
};

/** SubMesh normalized inside its bounding box (extents).
 */
struct NormalizedSubMesh {
    SubMesh submesh;
    math::Extents3 extents;

    typedef std::vector<NormalizedSubMesh> list;
};

// helper functions

std::uint32_t extraFlags(const Mesh &mesh);
std::uint32_t extraFlags(const Mesh *mesh);
std::uint32_t extraFlags(const Mesh::pointer &mesh);

math::Extents3 extents(const SubMesh &submesh);
math::Extents3 extents(const ConstSubMeshRange &smRange);
math::Extents3 extents(const Mesh &mesh);

/** Calculates geom-extents.
 *  NB: vertices must be in SDS to work properly.
 */
GeomExtents geomExtents(const math::Points3d &vertices);

/** Calculates geom-extents.
 *  NB: vertices must be in SDS to work properly.
 */
GeomExtents geomExtents(const SubMesh &submesh);

/** Calculates geom-extents.
 *  NB: vertices must be in SDS to work properly.
 */
GeomExtents geomExtents(const ConstSubMeshRange &smRange);

/** Calculates geom-extents.
 *  NB: vertices must be in SDS to work properly.
 */
GeomExtents geomExtents(const Mesh &mesh);

/** Calculates geom-extents. Convert to SDS system using provided convertor.
 */
GeomExtents geomExtents(const CsConvertor &conv, const SubMesh &submesh);

/** Calculates geom-extents. Convert to SDS system using provided convertor.
 */
GeomExtents geomExtents(const CsConvertor &conv
                        , const ConstSubMeshRange &smRange);

/** Calculates geom-extents. Convert to SDS system using provided convertor.
 */
GeomExtents geomExtents(const CsConvertor &conv, const Mesh &mesh);

/** Returns mesh and texture area for given submesh
 */
SubMeshArea area(const SubMesh &submesh);

/** Returns mesh and texture area for given submesh (only faces with all valid
 *  vertices are taken into account.
 *
 *  Prerequisite: mask.size() == submesh.vertices.size()
 */
SubMeshArea area(const SubMesh &submesh, const VertexMask &mask);

/** Simple area of 3D submesh.
 */
double area3d(const SubMesh &submesh);

/** Simple area of 3D submesh.
 */
double area3d(const SubMesh &submesh, const VertexMask &mask);

/** Low-level area calculator.
 */
SubMeshArea area(const math::Points3d &vertices, const Faces &faces
                 , const math::Points2d *tc, const Faces *facesTc
                 , const math::Points2d *etc
                 , const VertexMask *mask = nullptr);

/** Area of whole mesh.
 */
MeshArea area(const Mesh &mesh);

/** Area of whole mesh, with masks.
 */
MeshArea area(const Mesh &mesh, const VertexMasks &masks);

/** Generates external texture coordinates from vertices. SubMesh must be in
 *  spatial division SRS.
 *
 *  If external texture is not allowed any existing etc are removed.
 *
 * \param sm submesh to update (must be in SDS)
 * \param sdsExtents extents of tile in SDS
 * \param allowed flags whether exteranl texture is allowed for this submesh
 */
void generateEtc(SubMesh &sm, const math::Extents2 &sdsExtents
                 , bool allowed = true);

/** Generates external texture coordinates from vertices. Whole mesh must be in
 *  spatial division SRS.
 *
 *  If external texture is not allowed any existing etc are removed.
 *
 * \param mesh mesh to update (must be in SDS)
 * \param sdsExtents extents of tile in SDS
 * \param allowed flags whether exteranl texture is allowed for this submesh
 */
void generateEtc(Mesh &mesh, const math::Extents2 &sdsExtents
                 , bool allowed = true);

/** Updates coverage mask by rendering given submesh (must be in spatial
 *  division SRS).
 *
 * \param mesh coverage mask of this mesh is updated
 * \param sm submesh to render (must be in SDS)
 * \param sdsExtents extents of tile in SDS
 * \param smIndex submesh index
 */
void updateCoverage(Mesh &mesh, const SubMesh &sm
                    , const math::Extents2 &sdsExtents
                    , std::uint8_t smIndex = 1);

/** Generate coverage mask by rendering all submeshes (must be in spatial
 *  division SRS).
 *
 * \param mesh coverage mask of this mesh is updated
 * \param sdsExtents extents of tile in SDS
 */
void generateCoverage(Mesh &mesh, const math::Extents2 &sdsExtents);

/** Generates coverage to mesh mask.
 */
void generateMeshMask(MeshMask &mask, const Mesh &mesh
                      , const math::Extents2 &sdsExtents);

// IO
void saveMesh(std::ostream &out, const Mesh &mesh
              , const Atlas *atlas = nullptr);
void saveMesh(const boost::filesystem::path &path, const Mesh &mesh
              , const Atlas *atlas = nullptr);

Mesh loadMesh(std::istream &in, const boost::filesystem::path &path
              = "unknown");
Mesh loadMesh(const boost::filesystem::path &path);

void saveMesh(const storage::OStream::pointer &out, const Mesh &mesh
              , const Atlas *atlas = nullptr);
Mesh loadMesh(const storage::IStream::pointer &in);

/** Saves mesh as is.
 */
void saveMeshProper(std::ostream &out, const ConstSubMeshRange &submeshes
                    , const Atlas *atlas = nullptr
                    , bool compress = true);

void saveMeshProper(std::ostream &out, const Mesh &mesh
                    , const Atlas *atlas = nullptr
                    , bool compress = true);

void saveSubMeshAsObj(std::ostream &out, const SubMesh &sm
                      , std::size_t index, const Atlas *atlas = nullptr
                      , const std::string &matlib = "");

void saveSubMeshAsObj(const boost::filesystem::path &path, const SubMesh &sm
                      , std::size_t index, const Atlas *atlas = nullptr
                      , const std::string &matlib = "");

NormalizedSubMesh::list
loadMeshProperNormalized(std::istream &in
                         , const boost::filesystem::path &path);

multifile::Table readMeshTable(std::istream &is
                               , const boost::filesystem::path &path
                               = "unknown");
multifile::Table readMeshTable(const storage::IStream::pointer &in);

MeshMask loadMeshMask(std::istream &is
                      , const boost::filesystem::path &path
                      = "unknown");
MeshMask loadMeshMask(const boost::filesystem::path &path);
MeshMask loadMeshMask(const storage::IStream::pointer &in);

Mesh loadMeshFromObj(std::istream &is
                     , const boost::filesystem::path &path = "unknown");

// inlines

inline std::uint32_t extraFlags(const Mesh *mesh) {
    return mesh ? extraFlags(*mesh) : 0;
}

inline std::uint32_t extraFlags(const Mesh::pointer &mesh) {
    return extraFlags(mesh.get());
}

inline Mesh::Mesh(bool fullyCovered)
    : coverageMask(coverageOrder, fullyCovered)
{}

inline void Mesh::createCoverage(bool fullyCovered)
{
    coverageMask.recreate(coverageOrder, fullyCovered);
}

inline void MeshMask::createCoverage(bool fullyCovered)
{
    coverageMask.recreate(Mesh::coverageOrder, fullyCovered);
}

inline void SubMesh::cloneMetadataInto(SubMesh &dst) const
{
    dst.textureMode = textureMode;
    dst.textureLayer = textureLayer;
    dst.uvAreaScale = uvAreaScale;
    dst.surfaceReference = surfaceReference;
    dst.zIndex = dst.zIndex;
}

UTILITY_GENERATE_ENUM_IO(SubMesh::TextureMode,
    ((internal))
    ((external))
)

inline multifile::Table readMeshTable(const storage::IStream::pointer &in)
{
    return readMeshTable(*in, in->name());
}

inline void saveMesh(const storage::OStream::pointer &out, const Mesh &mesh
                     , const Atlas *atlas)
{
    return saveMesh(*out, mesh, atlas);
}

inline Mesh loadMesh(const storage::IStream::pointer &in)
{
    return loadMesh(*in, in->name());
}

inline MeshMask loadMeshMask(const storage::IStream::pointer &in)
{
    return loadMeshMask(*in, in->name());
}

inline double area3d(const SubMesh &submesh)
{
    return area(submesh.vertices, submesh.faces, nullptr, nullptr, nullptr)
        .mesh;
}

inline double area3d(const SubMesh &submesh, const VertexMask &mask)
{
    return area(submesh.vertices, submesh.faces, nullptr, nullptr, nullptr
                , &mask).mesh;
}

inline math::Extents3 extents(const Mesh &mesh) {
    return extents(ConstSubMeshRange(mesh.submeshes));
}

inline GeomExtents geomExtents(const Mesh &mesh) {
    return geomExtents(ConstSubMeshRange(mesh.submeshes));
}

inline GeomExtents geomExtents(const CsConvertor &conv, const Mesh &mesh) {
    return geomExtents(conv, ConstSubMeshRange(mesh.submeshes));
}

template <typename ...Args>
SubMeshRange submeshRange(Mesh &mesh, Args &&...args)
{
    return SubMeshRange(mesh.submeshes, std::forward<Args>(args)...);
}

template <typename ...Args>
SubMeshRange submeshRange(SubMesh::list &submeshes, Args &&...args)
{
    return SubMeshRange(submeshes, std::forward<Args>(args)...);
}

template <typename ...Args>
ConstSubMeshRange submeshRange(const Mesh &mesh, Args &&...args)
{
    return ConstSubMeshRange(mesh.submeshes, std::forward<Args>(args)...);
}

template <typename ...Args>
ConstSubMeshRange submeshRange(const SubMesh::list &submeshes
                               , Args &&...args)
{
    return ConstSubMeshRange(submeshes, std::forward<Args>(args)...);
}

inline void saveMeshProper(std::ostream &out, const Mesh &mesh
                           , const Atlas *atlas, bool compress)
{
    return saveMeshProper(out, submeshRange(mesh), atlas, compress);
}

} } // namespace vtslibs::vts

#endif // vtslibs_vts_mesh_hpp
