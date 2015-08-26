/**
 * \file vts/referenceframe.hpp
 * \author Vaclav Blazek <vaclav.blazek@citationtech.net>
 */

#ifndef vadstena_libs_vts_referenceframe_hpp_included_
#define vadstena_libs_vts_referenceframe_hpp_included_

#include <map>
#include <string>
#include <vector>

#include "utility/enum-io.hpp"

#include "math/geometry_core.hpp"

#include "geo/srsdef.hpp"

#include "../range.hpp"
#include "../ids.hpp"

namespace vadstena { namespace vts {

enum class Partitioning { bisection, manual };
enum class VerticalDatum { orthometric, ellipsoidal };

struct Sphereoid {
    double a;
    double b;

    Sphereoid(double a = 0, double b = 0) : a(a), b(b) {}
};

struct Srs {
    enum class Type { geographic, projected, cartesian };

    struct Modifiers {
        enum {
            adjustVertical = 0x01
        };
    };

    std::string comment;
    Type type;
    geo::SrsDefinition srsDef;
    int srsModifiers;

    boost::optional<Sphereoid> sphereoid;
    boost::optional<VerticalDatum> vdatum;
    boost::optional<geo::SrsDefinition> srsDefEllps;

    typedef std::map<std::string, Srs> map;

    Srs() : srsModifiers() {}
};

struct ReferenceFrame {
    struct Model {
        std::string physicalSrs;
        std::string navigationSrs;
        std::string publicSrs;
    };

    struct Division {
        struct Node {
            struct Id {
                Lod lod;
                unsigned int x;
                unsigned int y;

                Id(Lod lod = 0, unsigned int x = 0, unsigned int y = 0)
                    : lod(lod), x(x), y(y)
                {}

                bool operator<(const Id &tid) const;
            };

            Id id;
            std::string srs;
            math::Extents2 extents;
            Partitioning partitioning;

            typedef std::map<Id, Node> map;
        };

        math::Extents2 extents;
        Range<double> heightRange;
        unsigned int rootLod;
        unsigned int arity;
        Node::map nodes;
    };

    std::string id;
    std::string description;
    Division division;

    // parameters -- generic container?
    unsigned int metaBinaryOrder;

    typedef std::map<std::string, ReferenceFrame> map;
};

ReferenceFrame::map loadReferenceFrames(const boost::filesystem::path &path);

void saveReferenceFrames(const boost::filesystem::path &path
                         , const ReferenceFrame::map &rfs);

Srs::map loadSrs(const boost::filesystem::path &path);

void saveSrs(const boost::filesystem::path &path
             , const Srs::map &srs);

// enum IO stuff

UTILITY_GENERATE_ENUM_IO(VerticalDatum,
    ((orthometric))
    ((ellipsoidal))
)

UTILITY_GENERATE_ENUM_IO(Partitioning,
    ((bisection))
    ((manual))
)

UTILITY_GENERATE_ENUM_IO(Srs::Type,
    ((geographic))
    ((projected))
    ((cartesian))
)

// inlines

inline bool
ReferenceFrame::Division::Node::Id::operator<(const Id &id) const
{
    if (lod < id.lod) { return true; }
    else if (id.lod < lod) { return false; }

    if (x < id.x) { return true; }
    else if (id.x < x) { return false; }

    return y < id.y;
}

} } // namespace vadstena::vts

#endif // vadstena_libs_vts_referenceframe_hpp_included_
