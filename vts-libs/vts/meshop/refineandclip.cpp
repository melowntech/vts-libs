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
#include <map>

#include <boost/multi_index_container.hpp>
#include <boost/multi_index/ordered_index.hpp>
#include <boost/multi_index/member.hpp>

#include "dbglog/dbglog.hpp"

#include "utility/expect.hpp"

#include "../meshop.hpp"

namespace vtslibs { namespace vts {

namespace {

namespace bmi = boost::multi_index;

template <typename PointType>
struct Segment_ {
    const PointType &p1;
    const PointType &p2;

    PointType point(double t) const {
        return ((1.0 - t) * p1) + (t * p2);
    }

    Segment_(const PointType &p1, const PointType &p2) : p1(p1), p2(p2) {}
};

typedef Segment_<math::Point2d> Segment2;
typedef Segment_<math::Point3d> Segment3;

/** Plane defined as dot(p, normal) + c = 0
 */
struct ClipPlane {
    math::Point3d normal;
    double d;

    ClipPlane(double a, double b, double c, double d)
        : normal(a, b, c), d(d) {}
    ClipPlane() : d() {}

    double signedDistance(const math::Point3d &p) const {
        return boost::numeric::ublas::inner_prod(p, normal) + d;
    }

    double intersect(const math::Point3d &p1, const math::Point3d &p2) const {
        double dot1(boost::numeric::ublas::inner_prod(p1, normal));
        double dot2(boost::numeric::ublas::inner_prod(p2, normal));
        double den(dot1 - dot2);

        // line parallel with plane, return the midpoint
        if (std::abs(den) < 1e-10) {
            return 0.5;
        }
        return (dot1 + d) / den;
    }

    double intersect(const Segment3 &segment) const {
        return intersect(segment.p1, segment.p2);
    }

    typedef std::pair<const ClipPlane*, const ClipPlane*> const_range;
};

template <std::size_t N>
ClipPlane::const_range const_range(const ClipPlane (&planes)[N])
{
    return ClipPlane::const_range(std::begin(planes), std::end(planes));
}

const ClipPlane* begin(const ClipPlane::const_range &cr) { return cr.first; }
const ClipPlane* end(const ClipPlane::const_range &cr) { return cr.second; }

template<typename CharT, typename Traits>
inline std::basic_ostream<CharT, Traits>&
operator<<(std::basic_ostream<CharT, Traits> &os, const ClipPlane &cl)
{
    return os << "ClipPlane(" << cl.normal << ", " << cl.d << ")";
}

template<typename CharT, typename Traits, typename PointType>
inline std::basic_ostream<CharT, Traits>&
operator<<(std::basic_ostream<CharT, Traits> &os, const Segment_<PointType> &s)
{
    return os << "Segment(" << s.p1 << " -> " << s.p2 << ")";
}

typedef boost::optional<Face> OFace;

struct ClipFace {
    Face face;
    OFace faceTc;
    unsigned int origin;

    typedef std::vector<ClipFace> list;

    ClipFace() : origin() {}
    ClipFace(const ClipFace&) = default;
    ClipFace(const Face &face, unsigned int origin = 0)
        : face(face), origin(origin) {}
    ClipFace(Face::value_type a, Face::value_type b, Face::value_type c
             , unsigned int origin = 0)
        : face(a, b, c), origin(origin)
    {}
};

/** Maps point coordinates into list of points (one point is held only once)
 *  generated
 */
template <typename PointType>
class PointMapper {
public:
    template <typename Container>
    PointMapper(const Container &container)
        : points_(container.begin(), container.end())
    {}

    std::size_t add(const PointType &point)
    {
        auto res(mapping_.insert
                 (typename Mapping::value_type
                  (point, points_.size())));
        if (res.second) {
            // new point -> insert
            points_.push_back(point);
        }
        return res.first->second;
    }

    const std::vector<PointType>& points() const { return points_; }

    double distance(int v1, int v2) const {
        return boost::numeric::ublas::norm_2(points_[v1] - points_[v2]);
    }

private:
    typedef std::map<PointType, std::size_t> Mapping;
    std::vector<PointType> points_;
    Mapping mapping_;
};

class Clipper {
public:
    Clipper(const EnhancedSubMesh &mesh, const VertexMask &mask)
        : mesh_(mesh.mesh), fpmap_(mesh.projected), ftpmap_(mesh_.tc)
    {
        extractFaces();
        // TODO: apply mask
        (void) mask;
    }

    Clipper(const SubMesh &mesh, const VertexMask &mask)
        : mesh_(mesh), fpmap_(mesh.vertices), ftpmap_(mesh_.tc)
    {
        extractFaces();
        // TODO: apply mask
        (void) mask;
    }

    Clipper(const SubMesh &mesh, const math::Points3d &projected
            , const VertexMask &mask)
        : mesh_(mesh), fpmap_(projected), ftpmap_(mesh_.tc)
    {
        extractFaces();
        // TODO: apply mask
        (void) mask;
    }

    void refine(std::size_t faceCount);

    void clip(const ClipPlane &line);

    EnhancedSubMesh mesh(const MeshVertexConvertor *convertor = nullptr
                         , FaceOriginList * = nullptr);

    std::size_t faceCount() const { return faces_.size(); }

private:
    void extractFaces() {
        bool hasTc(!mesh_.facesTc.empty());
        for (std::size_t i(0), e(mesh_.faces.size()); i != e; ++i) {
            faces_.emplace_back(mesh_.faces[i], i);
            if (hasTc) { faces_.back().faceTc = mesh_.facesTc[i]; }
        }
    }

    const SubMesh &mesh_;

    ClipFace::list faces_;

    // face points map
    PointMapper<math::Point3d> fpmap_;

    // face texture points map
    PointMapper<math::Point2d> ftpmap_;
};

/** Use to uniformly map face vertices to fixed names.
 */
struct VMap {
    int a, b, c;

    template <typename Check>
    VMap(const bool (&inside)[3], Check check) {
        if (check(inside[0])) { a = 0; b = 1; c = 2; }
        else if (check(inside[1])) { a = 1; b = 2; c = 0; }
        else { a = 2; b = 0; c = 1; }
    }
};

void Clipper::clip(const ClipPlane &line)
{
    // LOG(debug) << "clip: " << line;
    ClipFace::list out;

    const auto &vertices(fpmap_.points());
    const auto &tc(ftpmap_.points());

    for (const auto cf : faces_) {
        const math::Point3d tri[3] = {
            vertices[cf.face[0]]
            , vertices[cf.face[1]]
            , vertices[cf.face[2]]
        };

        const bool inside[3] = {
            (line.signedDistance(tri[0]) >= .0)
            , (line.signedDistance(tri[1]) >= .0)
            , (line.signedDistance(tri[2]) >= .0)
        };

        // LOG(debug) << std::fixed << "cutting face " << tri << ":"
        //            << "\n    " << tri[0] << ", " << inside[0]
        //            << "\n    " << tri[1] << ", " << inside[1]
        //            << "\n    " << tri[2] << ", " << inside[2];

        int count(inside[0] + inside[1] + inside[2]);
        if (!count) {
            // LOG(debug) << "    -> fully outside";
            // whole face is outside
            continue;
        }

        if (count == 3) {
            // whole face is inside
            // LOG(debug) << "    -> fully inside";
            out.push_back(cf);
            continue;
        }

        // oneInside: true: one inside, false: one outside
        bool oneInside(count == 1);

        VMap vm(inside, (oneInside ? ([](bool i) { return i; })
                         : ([](bool i) { return !i; })));

        double t1, t2;

        /* mesh face */ {
            Segment3 s1(tri[vm.a], tri[vm.b]);
            Segment3 s2(tri[vm.c], tri[vm.a]);

            // intersect segments with both lines
            t1 = line.intersect(s1);
            t2 = line.intersect(s2);

            // calculate new point
            auto p1(s1.point(t1));
            auto p2(s2.point(t2));

            // LOG(debug) << "    " << s1 << " -> " << t1 << " -> " << p1;
            // LOG(debug) << "    " << s2 << " -> " << t2 << " -> " << p2;

            // and new (projected) vertices
            auto vi1(fpmap_.add(p1));
            auto vi2(fpmap_.add(p2));

            if (oneInside) {
                // one vertex inside: just one face:
                out.emplace_back(cf.face[vm.a], vi1, vi2, cf.origin);

                // LOG(debug)
                //     << std::fixed
                //     << "    -> one vertex inside, new face:"
                //     << "\n    " << tri[vm.a]
                //     << "\n    " << p1
                //     << "\n    " << p2;

            } else {
                // one vertex outside: two new faces
                out.emplace_back(vi1, cf.face[vm.b], cf.face[vm.c]
                                 , cf.origin);
                out.emplace_back(vi1, cf.face[vm.c], vi2, cf.origin);

                // LOG(debug)
                //     << std::fixed
                //     << "\n    -> one vertex outside, new faces:"
                //     << "\n    " << p1
                //     << "\n    " << tri[vm.b]
                //     << "\n    " << tri[vm.c]
                //     << "\n    " << p1
                //     << "\n    " << tri[vm.c]
                //     << "\n    " << p2;
            }
        }

        if (cf.faceTc) {
            // texture face
            auto face(*cf.faceTc);

            // LOG(debug) << "cutting texture face " << face << ":";
            // LOG(debug)
            //     << std::fixed << "    " << tc[face[0]] << ", " << inside[0];
            // LOG(debug)
            //     << std::fixed << "    " << tc[face[1]] << ", " << inside[1];
            // LOG(debug)
            //     << std::fixed << "    " << tc[face[2]] << ", " << inside[2];

            // calculate new point
            auto tp1(Segment2(tc[face[vm.a]], tc[face[vm.b]]).point(t1));
            auto tp2(Segment2(tc[face[vm.c]], tc[face[vm.a]]).point(t2));

            auto ti1(ftpmap_.add(tp1));
            auto ti2(ftpmap_.add(tp2));

            if (oneInside) {
                // one vertex inside: just one face:
                out.back().faceTc = Face(face[vm.a], ti1, ti2);

                // LOG(debug)
                //     << std::fixed
                //     << "    -> one vertex inside, new face "
                //     << *out.back().faceTc << ":"
                //     << "\n    " << tc[face[vm.a]]
                //     << "\n    " << tp1
                //     << "\n    " << tp2;
            } else {
                // one vertex outside: two new faces
                out[out.size() - 2].faceTc = Face(ti1, face[vm.b], face[vm.c]);
                out[out.size() - 1].faceTc = Face(ti1, face[vm.c], ti2);

                // LOG(debug)
                //     << std::fixed
                //     << "    -> one vertex outside, new faces "
                //     << *out[out.size() - 2].faceTc << ", "
                //     << *out[out.size() - 1].faceTc << ":"
                //     << "\n    " << tp1
                //     << "\n    " << tc[face[vm.b]]
                //     << "\n    " << tc[face[vm.c]]
                //     << "\n    " << tp1
                //     << "\n    " << tc[face[vm.c]]
                //     << "\n    " << tp2;
            }
        }
    }

    out.swap(faces_);
}

void Clipper::refine(std::size_t faceCount)
{
    if (faceCount <= faces_.size()) { return; }
    LOG(info2) << "Refining " << faces_.size() << " faces to " << faceCount
               << " faces.";

#if 0
    typedef PointMapper<math::Point3d> Vertices;
#endif

    /** Edge key, keys is held automatically sorted.
     */
    struct EdgeKey {
        int v1;
        int v2;

        EdgeKey(int v1, int v2)
            : v1(std::min(v1, v2))
            , v2(std::max(v1, v2))
        {}

        bool operator<(const EdgeKey &o) const {
            if (v1 < o.v1) {
                return true;
            } else if (o.v1 < v1) {
                return false;
            }
            return v2 < o.v2;
        }
    };

    struct EdgeFace {
        int face;
        int i1;

        EdgeFace(int face, int i1) : face(face), i1(i1) {}

        typedef std::vector<EdgeFace> list;
    };

    struct Edge {
        EdgeKey key;
        double length;
        EdgeFace::list faces;

        Edge() = default;

        Edge(const EdgeKey &key, double length)
            : key(key), length(length)
        {}

        std::tuple<Edge, Edge> split(int vh) const {
            return std::tuple<Edge, Edge>
                (Edge(EdgeKey(key.v1, vh), length / 2.0)
                 , Edge(EdgeKey(vh, key.v2), length / 2.0));
        }

        bool operator<(const Edge &o) const {
            return key < o.key;
        }

        bool has(int v) const {
            return (key.v1 == v) || (key.v2 == v);
        }
    };

    struct KeyIdx {};
    struct LengthIdx {};
    typedef boost::multi_index_container<
        Edge
        , bmi::indexed_by<
              bmi::ordered_unique<
                    bmi::tag<KeyIdx>
                    , BOOST_MULTI_INDEX_MEMBER(Edge, EdgeKey, key)
                  >

              , bmi::ordered_non_unique<
                    bmi::tag<LengthIdx>
                    , BOOST_MULTI_INDEX_MEMBER(Edge, double, length)
                    , std::greater<double>
                    >
              >

        > Edges;

    ClipFace::list &faces(faces_);
    Edges edges;

    auto addEdge([&](const EdgeKey &key, int findex, int i1
                     , int oldFindex)
    {
        auto fedges(edges.find(key));
        if (fedges == edges.end()) {
            // adding new edge
            fedges = edges.insert
                (Edge(key, fpmap_.distance(key.v1, key.v2))).first;
            const_cast<EdgeFace::list&>
                (fedges->faces).emplace_back(findex, i1);
            return;
        }

        // updating existing edge
        auto &faces(const_cast<EdgeFace::list&>(fedges->faces));
        if (oldFindex >= 0) {
            auto ffaces(std::find_if(faces.begin(), faces.end()
                                     , [&](const EdgeFace &ef)
            {
                return (ef.face == oldFindex);
            }));

            if (ffaces != faces.end()) {
                // LOG(debug) << "    replacing face (" << ffaces->face
                //            << ", " << ffaces->i1
                //            << ") with face (" << findex << ", " << i1
                //            << ")";
                // found -> replace
                ffaces->face = findex;
                ffaces->i1 = i1;
                return;
            }

            // not found -> fall through
        }

        // not found or not replacing -> add
        faces.emplace_back(findex, i1);
    });

    {
        std::size_t findex(0);

        for (const auto &cf : faces) {
            const auto &face(cf.face);

            auto addEdgeFrom([&](int i1)
            {
                addEdge(EdgeKey(face[i1], face[(i1 + 1) % 3])
                        , findex, i1, -1);
            });

            addEdgeFrom(0);
            addEdgeFrom(1);
            addEdgeFrom(2);
            ++findex;
        }
    }

    const auto &tc(ftpmap_.points());
    const auto &vertices(fpmap_.points());

    auto &idx(edges.get<LengthIdx>());
    while (faces.size() < faceCount) {
        auto iedges(idx.begin());

        const auto &edge(*iedges);

        // split edge in half and remember index
        auto vh(fpmap_.add
                ((vertices[edge.key.v1] + vertices[edge.key.v2]) / 2.0));

        // generate half edges
        auto halves(edge.split(vh));
        auto &e1(std::get<0>(halves));
        auto &e2(std::get<1>(halves));

        // LOG(debug) << "Splitting (" << edge.key.v1 << ", " << edge.key.v2
        //            << ") into (" << e1.key.v1 << ", " << e1.key.v2
        //            << ") and (" <<  e2.key.v1 << ", " << e2.key.v2 << ").";

        for (const auto &ef : edge.faces) {
            // get old and new face index
            int fi1(ef.face);
            int fi2(faces.size());

            // clone face to second face
            faces.push_back(faces[fi1]);

            // get reference to first face
            Face &face1(faces[fi1].face);
            // get reference to new (second) face
            Face &face2(faces.back().face);

            int i1(ef.i1);
            int i2((ef.i1 + 1) % 3);
            int i3((ef.i1 + 2) % 3);

            // LOG(debug) << "    before split: " << face1 << ", " << face2
            //            << " (" << i1 << ")";
            // replace end edge vertices with half-way vertex
            face1(i2) = vh;
            face2(i1) = vh;
            // LOG(debug) << "    after split: " << face1 << ", " << face2;

            // add new faces to half-edges (since edge is oriented from lower to
            // higher index we have to check which way the edge is)
            if (e1.has(face1[i1])) {
                e1.faces.emplace_back(fi1, i1);
                e2.faces.emplace_back(fi2, i1);
            } else {
                e1.faces.emplace_back(fi2, i1);
                e2.faces.emplace_back(fi1, i1);
            }

            auto v2(face2(i2));
            auto v3(face2(i3));

            // create 3rd new edge that is shared between two new triangles
            {
                EdgeKey e3key(vh, v3);
                Edge e3(e3key, fpmap_.distance(e3key.v1, e3key.v2));
                // LOG(debug) << "    added edge ("
                           // << e3key.v1 << ", " << e3key.v2 << ")";
                e3.faces.emplace_back(fi1, i2);
                e3.faces.emplace_back(fi2, i3);
                edges.insert(e3);
            }

            // replace fi1 with fi2 in edge(i2, i3)
            // LOG(debug) << "    remapping edge ("
            //            << v2 << ", " << v3 << ") from " << fi1
            //            << " to " << fi2;
            addEdge(EdgeKey(v2, v3), fi2, i2, fi1);

            if (!tc.empty()) {
                // split texturing face as well
                Face &tf1(*faces[fi1].faceTc);
                Face &tf2(*faces[fi2].faceTc);

                // split edge
                auto th(ftpmap_.add((tc[tf1(i1)] + tc[tf1(i2)]) / 2.0));

                // assign
                tf1(i2) = th;
                tf2(i1) = th;
            }
        }

        // add new edges
        edges.insert(e1);
        edges.insert(e2);

        // and finally remove original edge
        idx.erase(iedges);
    }
}

EnhancedSubMesh Clipper::mesh(const MeshVertexConvertor *convertor
                              , FaceOriginList *faceOrigin)
{
    /** Generate new mesh.
     */
    struct Filter {
        const SubMesh &original;
        const ClipFace::list &faces;
        const math::Points3d &vertices;
        const math::Points3d &projected;
        const math::Points2d &tc;
        const MeshVertexConvertor *convertor;
        FaceOriginList *faceOrigin;

        std::vector<int> vertexMap;
        std::vector<int> tcMap;

        EnhancedSubMesh emesh;
        SubMesh &mesh;

        Filter(const SubMesh &original, const ClipFace::list &faces
               , const math::Points3d &projected
               , const math::Points2d &tc
               , const MeshVertexConvertor *convertor
               , FaceOriginList *faceOrigin)
            : original(original), faces(faces), vertices(original.vertices)
            , projected(projected), tc(tc), convertor(convertor)
            , faceOrigin(faceOrigin)
            , vertexMap(projected.size(), -1)
            , tcMap(tc.size(), -1)
            , mesh(emesh.mesh)
        {
            // clone metadata into output mesh
            original.cloneMetadataInto(mesh);
            for (const auto &cf : faces) {
                addFace(cf);
            }
        }

        void addFace(const ClipFace &cf) {
            mesh.faces.emplace_back(addVertex(cf.face(0))
                                    , addVertex(cf.face(1))
                                    , addVertex(cf.face(2)));
            // LOG(debug) << "Added tc face: " << cf.face << " -> "
            //            << mesh.faces.back();

            if (cf.faceTc) {
                mesh.facesTc.emplace_back(addTc((*cf.faceTc)(0))
                                          , addTc((*cf.faceTc)(1))
                                          , addTc((*cf.faceTc)(2)));
                // LOG(debug) << "Added tc face: " << *cf.faceTc << " -> "
                //            << mesh.facesTc.back();
            }

            if (faceOrigin) { faceOrigin->push_back(cf.origin); }
        }

        std::size_t addVertex(std::size_t i) {
            auto &m(vertexMap[i]);
            if (m < 0) {
                // new vertex
                m = mesh.vertices.size();

                const auto newPoint(i >= vertices.size());
                const auto generateEtc(convertor && !original.etc.empty());
                const auto &v(projected[i]);

                if (newPoint) {
                    // must unproject
                    mesh.vertices.push_back(convertor
                                            ? convertor->vertex(v)
                                            : v);

                    // etc
                    if (generateEtc) {
                        mesh.etc.push_back(convertor->etc(v));
                        // LOG(debug) << v << ": etc from vertex: " << v << " -> "
                        //            << mesh.etc.back();
                    }
                } else {
                    // use original
                    mesh.vertices.push_back(vertices[i]);

                    if (generateEtc) {
                        mesh.etc.push_back(convertor->etc(original.etc[i]));
                        // LOG(debug) << v << ": etc from old: "
                        //            << original.etc[i]
                        //            << " -> " << mesh.etc.back();
                    }
                }

                // remember projected vertex
                emesh.projected.push_back(v);
            }
            return m;
        }

        int addTc(int i) {
            auto &m(tcMap[i]);
            if (m < 0) {
                // new vertex
                m = mesh.tc.size();
                mesh.tc.push_back(tc[i]);
            }
            return m;
        }
    };

    // run the machinery
    return Filter(mesh_, faces_, fpmap_.points()
                  , ftpmap_.points()
                  , convertor, faceOrigin).emesh;
}

} // namespace

EnhancedSubMesh clipAndRefine(const EnhancedSubMesh &mesh
                              , const math::Extents2 &projectedExtents
                              , const MeshVertexConvertor &convertor
                              , const VertexMask &mask)
{
    LOG(debug) << std::fixed << "Clipping mesh to: " << projectedExtents;
    const ClipPlane clipPlanes[4] = {
        { 1.,  .0, .0, -projectedExtents.ll(0) }
        , { -1., .0, .0, projectedExtents.ur(0) }
        , { .0,  1., .0, -projectedExtents.ll(1) }
        , { 0., -1., .0, projectedExtents.ur(1) }
    };

    Clipper clipper(mesh, mask);

    // clip by clipping planes
    for (const auto &cp : clipPlanes) { clipper.clip(cp); }

    // refine clipped mesh to requested number of faces
    clipper.refine(convertor.refineToFaceCount(clipper.faceCount()));

    return clipper.mesh(&convertor);
}

SubMesh clip(const SubMesh &mesh, const math::Points3d &projected
             , const ClipPlane::const_range &clipPlanes
             , const VertexMask &mask, FaceOriginList *faceOrigin)
{
    Clipper clipper(mesh, projected, mask);

    for (const auto &cp : clipPlanes) { clipper.clip(cp); }

    // extract enhanced mesh from clipper (use no convertor)
    auto emesh(clipper.mesh(nullptr, faceOrigin));
    (void) faceOrigin;
    // swap vertices with projected vertices (we are working in local system
    emesh.mesh.vertices.swap(emesh.projected);

    // get output mesh
    const auto &out(emesh.mesh);

    LOG(debug) << "Mesh clipped: vertices="
               << projected.size() << "->" << out.vertices.size()
               << ", tc=" << mesh.tc.size() << "->" << out.tc.size()
               << ", etc=" << mesh.etc.size()
               << "->" << out.etc.size()
               << ", faces=" << mesh.faces.size()
               << "->" << out.faces.size()
               << ", facesTc=" << mesh.facesTc.size()
               << "->" << out.facesTc.size()
               << ".";

    // and return
    return out;
}

SubMesh clip(const SubMesh &mesh, const math::Points3d &projected
             , const math::Extents2 &projectedExtents
             , const VertexMask &mask, FaceOriginList *faceOrigin)
{
    LOG(debug) << std::fixed << "Clipping mesh to: " << projectedExtents;
    const ClipPlane clipPlanes[4] = {
        { 1.,  .0, .0, -projectedExtents.ll(0) }
        , { -1., .0, .0, projectedExtents.ur(0) }
        , { .0,  1., .0, -projectedExtents.ll(1) }
        , { 0., -1., .0, projectedExtents.ur(1) }
    };
    return clip(mesh, projected, const_range(clipPlanes), mask, faceOrigin);
}

SubMesh clip(const SubMesh &mesh, const math::Points3d &projected
             , const math::Extent &projectedVerticalExtent
             , const VertexMask &mask, FaceOriginList *faceOrigin)
{
    LOG(debug)
        << std::fixed << "Clipping mesh to: " << projectedVerticalExtent;
    const ClipPlane clipPlanes[4] = {
        { 0.,  .0, 1., -projectedVerticalExtent.l }
        , { 0.,  .0, -1., projectedVerticalExtent.r }
    };
    return clip(mesh, projected, const_range(clipPlanes), mask, faceOrigin);
}

EnhancedSubMesh clip(const SubMesh &mesh, const math::Points3d &projected
                     , const math::Extents2 &projectedExtents
                     , const MeshVertexConvertor &convertor
                     , const VertexMask &mask
                     , FaceOriginList *faceOrigin)
{
    LOG(debug) << std::fixed << "Clipping mesh to: " << projectedExtents;
    const ClipPlane clipPlanes[4] = {
        { 1.,  .0, .0, -projectedExtents.ll(0) }
        , { -1., .0, .0, projectedExtents.ur(0) }
        , { .0,  1., .0, -projectedExtents.ll(1) }
        , { 0., -1., .0, projectedExtents.ur(1) }
    };

    Clipper clipper(mesh, projected, mask);

    for (const auto &cp : clipPlanes) { clipper.clip(cp); }

    return clipper.mesh(&convertor, faceOrigin);
}

namespace {

template <typename Container>
const Container* nonempty(const Container &c)
{
    return c.empty() ? nullptr : &c;
}

} // namespace

/** Compute enhanced submesh area.
 */
SubMeshArea area(const EnhancedSubMesh &submesh, const VertexMask &mask)
{
    return area(submesh.projected, submesh.mesh.faces
                , nonempty(submesh.mesh.tc)
                , nonempty(submesh.mesh.facesTc)
                , nonempty(submesh.mesh.etc)
                , &mask);
}

/** Compute enhanced mesh area.
 */
MeshArea area(const EnhancedSubMesh::list &mesh, const VertexMasks &masks)
{
    utility::expect((mesh.size() == masks.size())
                    , "Number of submeshes (%d) different "
                    "from number of masks (%d)."
                    , mesh.size(), masks.size());

    MeshArea out;
    auto imask(masks.begin());
    for (const auto &sm : mesh) {
        out.submeshes.push_back(area(sm, *imask++));
        out.mesh += out.submeshes.back().mesh;
    }
    return out;
}

} } // namespace vtslibs::vts
