#include "dbglog/dbglog.hpp"

#include "./refineandclip.hpp"

namespace vadstena { namespace vts {

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

struct ClipLine {
    math::Point2d normal;
    double c;

    ClipLine(double a, double b, double c) : normal(a, b), c(c) {}
    ClipLine() : c() {}

    double signedDistance(const math::Point2d &p) const {
        return boost::numeric::ublas::inner_prod(p, normal) + c;
    }

    double intersect(const math::Point3d &p1, const math::Point3d &p2) const {
        double dot1(boost::numeric::ublas::inner_prod(p1, normal));
        double dot2(boost::numeric::ublas::inner_prod(p2, normal));
        double den(dot1 - dot2);

        // line parallel with plane, return the midpoint
        if (std::abs(den) < 1e-10) {
            return 0.5;
        }
        return (dot1 + c) / den;
    }

    double intersect(const Segment2 &segment) const {
        return intersect(segment.p1, segment.p2);
    }
};

template<typename CharT, typename Traits>
inline std::basic_ostream<CharT, Traits>&
operator<<(std::basic_ostream<CharT, Traits> &os, const ClipLine &cl)
{
    return os << "ClipLine(" << cl.normal << ", " << cl.c << ")";
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
};

class Clipper {
public:
    Clipper(const EnhancedSubMesh &mesh)
        : mesh_(mesh)
    {
        extractFaces();
    }

    void refine(int lodDiff);

    void clip(const ClipLine &line);

    EnhancedSubMesh mesh() { finalize(); return mesh_; }

private:
    void extractFaces() {
        bool hasTc(!mesh_.mesh.facesTc.empty());
        for (std::size_t i(0), e(mesh_.mesh.faces.size()); i != e; ++i) {
            faces_.emplace_back(mesh_.mesh.faces[i]);
            if (hasTc) { faces_.back().faceTc = mesh_.mesh.facesTc[i]; }
        }
    }

    void finalize();

    EnhancedSubMesh mesh_;
    ClipFace::list faces_;
};

class PointMapper {
public:
    PointMapper(math::Points3d &points) : points_(points) {}

    std::size_t add(const math::Point3d &point);

private:
    math::Points3d &points_;
    typedef std::map<math::Point2d, std::size_t> Mapping;
    Mapping mapping_;
};

inline std::size_t PointMapper::add(const math::Point3d &p)
{
    auto res(mapping_.insert(Mapping::value_type(p, points_.size())));
    if (res.second) {
        // new point -> insert
        points_.push_back(p);
    }
    return res.first->second;
}

void Clipper::clip(const ClipLine &line)
{
    LOG(info4) << "clip: " << line;
    ClipFace::list out;

    PointMapper pmap(mesh_.projected);

    for (const auto cf : faces_) {
        const math::Point2d tri[3] = {
            mesh_.projected[cf.face[0]]
            , mesh_.projected[cf.face[1]]
            , mesh_.projected[cf.face[2]]
        };

        const bool inside[3] = {
            (line.signedDistance(tri[0]) >= .0)
            , (line.signedDistance(tri[1]) >= .0)
            , (line.signedDistance(tri[2]) >= .0)
        };

        LOG(info4) << "face:";
        LOG(info4) << "    " << tri[0] << ", " << inside[0];
        LOG(info4) << "    " << tri[1] << ", " << inside[1];
        LOG(info4) << "    " << tri[2] << ", " << inside[2];

        int count(inside[0] + inside[1] + inside[2]);
        if (!count) {
            // whole face is outside
            continue;
        }

        if (count == 3) {
            // whole face is inside
            out.push_back(cf);
            continue;
        }

        struct Segment {
            const math::Point3d &p1;
            const math::Point3d &p2;
        };

        if (count == 1) {
            LOG(info4) << "cut one";
            // one vertex is inside -> move those two to proper place

            int a, b, c;
            if (inside[0]) { a = 0; b = 1; c = 2; }
            else if (inside[1]) { a = 1; b = 2; c = 0; }
            else { a = 2; b = 0; c = 1; }

            Segment2 s1(tri[a], tri[b]);
            Segment2 s2(tri[c], tri[a]);

            auto t1(line.intersect(s1));
            auto t2(line.intersect(s2));

            auto p1(s1.point(t1));
            auto p2(s2.point(t2));

            LOG(info4) << "    " << s1 << " -> " << t1 << " -> " << p1;
            LOG(info4) << "    " << s2 << " -> " << t2 << " -> " << p2;

            // real mesh coordinates will be generated from projected
            auto i1(pmap.add(p1));
            auto i2(pmap.add(p2));

            Face nface(cf.face[a], i1, i2);
            out.emplace_back(nface);

            continue;
        }

        LOG(info4) << "cut two";
    }

    out.swap(faces_);
}

void Clipper::finalize()
{
    // TODO: implement me
}

void Clipper::refine(int lodDiff)
{
    if (!lodDiff) { return; }
    (void) lodDiff;
    // TODO: implement me
}

EnhancedSubMesh refineAndClip(const EnhancedSubMesh &mesh
                              , const math::Extents2 &projectedExtents
                              , storage::Lod lodDiff)
{
    LOG(info4) << std::fixed << "projectedExtents: " << projectedExtents;
    const ClipLine clipLines[4] = {
        { 1.,  .0, -projectedExtents.ll(0) }
        , { -1., .0, projectedExtents.ur(0) }
        , { .0,  1., -projectedExtents.ll(1) }
        , { 0., -1., projectedExtents.ur(1) }
    };

    Clipper clipper(mesh);

    clipper.refine(lodDiff);

    for (const auto &cl : clipLines) { clipper.clip(cl); }

    return clipper.mesh();
}

} } // namespace vadstena::vts
