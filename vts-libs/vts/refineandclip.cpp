#include "dbglog/dbglog.hpp"

#include "./refineandclip.hpp"

namespace vadstena { namespace vts {

namespace {

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

    double signedDistance(const math::Point2d &p) const {
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
};

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

    typedef std::vector<ClipFace> list;

    ClipFace(const Face &face) : face(face) {}
    ClipFace(Face::value_type a, Face::value_type b, Face::value_type c)
        : face(a, b, c)
    {}
};

/** Maps point coordinates into list of points (one point is held only once)
 *  generated
 */
template <typename PointType>
class PointMapper {
public:
    PointMapper(const std::vector<PointType> &points)
        : points_(points)
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

private:
    typedef std::map<PointType, std::size_t> Mapping;
    std::vector<PointType> points_;
    Mapping mapping_;
};

class Clipper {
public:
    Clipper(const EnhancedSubMesh &mesh)
        : mesh_(mesh)
        , fpmap_(mesh_.projected), ftpmap_(mesh_.mesh.tc)
    {
        extractFaces();
    }

    void refine(int lodDiff);

    void clip(const ClipPlane &line);

    EnhancedSubMesh mesh(const MeshVertexConvertor &convertor);

private:
    void extractFaces() {
        bool hasTc(!mesh_.mesh.facesTc.empty());
        for (std::size_t i(0), e(mesh_.mesh.faces.size()); i != e; ++i) {
            faces_.emplace_back(mesh_.mesh.faces[i]);
            if (hasTc) { faces_.back().faceTc = mesh_.mesh.facesTc[i]; }
        }
    }

    const EnhancedSubMesh &mesh_;

    ClipFace::list faces_;

    // face points and  map
    PointMapper<math::Point3d> fpmap_;

    // face texture points and map
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
    LOG(debug) << "clip: " << line;
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

        LOG(debug) << "cutting face:";
        LOG(debug) << "    " << tri[0] << ", " << inside[0];
        LOG(debug) << "    " << tri[1] << ", " << inside[1];
        LOG(debug) << "    " << tri[2] << ", " << inside[2];

        int count(inside[0] + inside[1] + inside[2]);
        if (!count) {
            LOG(debug) << "    -> fully outside";
            // whole face is outside
            continue;
        }

        if (count == 3) {
            // whole face is inside
            LOG(debug) << "    -> fully inside";
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

            LOG(debug) << "    " << s1 << " -> " << t1 << " -> " << p1;
            LOG(debug) << "    " << s2 << " -> " << t2 << " -> " << p2;

            // and new (projected) vertices
            auto vi1(fpmap_.add(p1));
            auto vi2(fpmap_.add(p2));

            if (oneInside) {
                // one vertex inside: just one face:
                out.emplace_back(cf.face[vm.a], vi1, vi2);

                LOG(debug) << "    -> one vertex inside, new face:";
                LOG(debug) << "    " << tri[vm.a];
                LOG(debug) << "    " << p1;
                LOG(debug) << "    " << p2;

            } else {
                // one vertex outside: two new faces
                out.emplace_back(vi1, cf.face[vm.b], cf.face[vm.c]);
                out.emplace_back(vi1, cf.face[vm.c], vi2);

                LOG(debug) << "    -> one vertex outside, new faces:";
                LOG(debug) << "    " << p1;
                LOG(debug) << "    " << tri[vm.b];
                LOG(debug) << "    " << tri[vm.c];
                LOG(debug) << "    " << p1;
                LOG(debug) << "    " << tri[vm.c];
                LOG(debug) << "    " << p2;
            }
        }

        if (cf.faceTc) {
            // texture face
            auto face(*cf.faceTc);

            LOG(debug) << "cutting texture face:";
            LOG(debug) << "    " << tc[face[0]] << ", " << inside[0];
            LOG(debug) << "    " << tc[face[1]] << ", " << inside[1];
            LOG(debug) << "    " << tc[face[2]] << ", " << inside[2];

            // calculate new point
            auto tp1(Segment2(tc[face[vm.a]], tc[face[vm.b]]).point(t1));
            auto tp2(Segment2(tc[face[vm.c]], tc[face[vm.a]]).point(t2));

            auto ti1(ftpmap_.add(tp1));
            auto ti2(ftpmap_.add(tp2));

            if (oneInside) {
                // one vertex inside: just one face:
                out.back().faceTc = Face(face[vm.a], ti1, ti2);

                LOG(debug) << "    -> one vertex inside, new face:";
                LOG(debug) << "    " << tc[face[vm.a]];
                LOG(debug) << "    " << tp1;
                LOG(debug) << "    " << tp2;
            } else {
                // one vertex outside: two new faces
                out[out.size() - 2].faceTc = Face(ti1, face[vm.b], face[vm.c]);
                out[out.size() - 1].faceTc = Face(ti1, face[vm.c], ti2);

                LOG(debug) << "    -> one vertex outside, new faces:";
                LOG(debug) << "    " << tp1;
                LOG(debug) << "    " << tc[face[vm.b]];
                LOG(debug) << "    " << tc[face[vm.c]];
                LOG(debug) << "    " << tp1;
                LOG(debug) << "    " << tc[face[vm.c]];
                LOG(debug) << "    " << tp2;
            }
        }
    }

    out.swap(faces_);
}

void Clipper::refine(int lodDiff)
{
    if (!lodDiff) { return; }
    (void) lodDiff;
    // TODO: implement me
}

EnhancedSubMesh Clipper::mesh(const MeshVertexConvertor &convertor)
{
    /** Generate new mesh.
     */
    struct Filter {
        const SubMesh &original;
        const ClipFace::list &faces;
        const math::Points3d &vertices;
        const math::Points3d &projected;
        const math::Points2d &tc;
        const MeshVertexConvertor &convertor;

        std::vector<int> vertexMap;
        std::vector<int> tcMap;

        EnhancedSubMesh emesh;
        SubMesh &mesh;

        Filter(const SubMesh &original, const ClipFace::list &faces
               , const math::Points3d &projected, const math::Points2d &tc
               , const MeshVertexConvertor &convertor)
            : original(original), faces(faces), vertices(original.vertices)
            , projected(projected), tc(tc), convertor(convertor)
            , vertexMap(projected.size(), -1)
            , tcMap(tc.size(), -1)
            , mesh(emesh.mesh)
        {
            for (const auto &cf : faces) {
                addFace(cf);
            }
        }

        void addFace(const ClipFace &cf) {
            mesh.faces.emplace_back(addVertex(cf.face(0))
                                    , addVertex(cf.face(1))
                                    , addVertex(cf.face(2)));
            if (cf.faceTc) {
                mesh.facesTc.emplace_back(addTc((*cf.faceTc)(0))
                                          , addTc((*cf.faceTc)(1))
                                          , addTc((*cf.faceTc)(2)));
            }
        }

        std::size_t addVertex(std::size_t i) {
            auto &m(vertexMap[i]);
            if (m < 0) {
                // new vertex
                m = mesh.vertices.size();

                const auto newPoint(i >= vertices.size());
                const auto generateEtc(!original.etc.empty());
                const auto &v(projected[i]);

                if (newPoint) {
                    // must unproject
                    mesh.vertices.push_back(convertor.vertex(v));

                    // etc
                    if (generateEtc) {
                        mesh.etc.push_back(convertor.etc(original.etc[i]));
                    }
                } else {
                    // use original
                    mesh.vertices.push_back(vertices[i]);

                    if (generateEtc) {
                        mesh.etc.push_back(v);
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
    return Filter(mesh_.mesh, faces_, fpmap_.points(), ftpmap_.points()
                  , convertor).emesh;
}

} // namespace

EnhancedSubMesh refineAndClip(const EnhancedSubMesh &mesh
                              , const math::Extents2 &projectedExtents
                              , storage::Lod lodDiff
                              , const MeshVertexConvertor &convertor)
{
    LOG(debug) << std::fixed << "Clipping mesh to: " << projectedExtents;
    const ClipPlane clipPlanes[4] = {
        { 1.,  .0, .0, -projectedExtents.ll(0) }
        , { -1., .0, .0, projectedExtents.ur(0) }
        , { .0,  1., .0, -projectedExtents.ll(1) }
        , { 0., -1., .0, projectedExtents.ur(1) }
    };

    Clipper clipper(mesh);

    clipper.refine(lodDiff);

    for (const auto &cp : clipPlanes) { clipper.clip(cp); }

    return clipper.mesh(convertor);
}

} } // namespace vadstena::vts
