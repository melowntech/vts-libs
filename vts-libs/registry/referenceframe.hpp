/**
 * \file registry/referenceframe.hpp
 * \author Vaclav Blazek <vaclav.blazek@citationtech.net>
 */

#ifndef vadstena_libs_registry_referenceframe_hpp_included_
#define vadstena_libs_registry_referenceframe_hpp_included_

#include <set>
#include <map>
#include <string>
#include <vector>
#include <new>

#include "dbglog/dbglog.hpp"

#include "utility/enum-io.hpp"

#include "math/geometry_core.hpp"

#include "geo/srsdef.hpp"

#include "../storage/error.hpp"
#include "../storage/lod.hpp"
#include "../storage/range.hpp"
#include "../storage/credits.hpp"

#include "./dict.hpp"

namespace vadstena { namespace registry {

using storage::Lod;
using storage::LodRange;
using storage::CreditId;
using storage::CreditIds;

typedef std::set<std::string> StringIdSet;
typedef std::set<int> IdSet;

typedef math::Extents2_<unsigned int> TileRange;

enum class PartitioningMode { bisection, manual, none };
enum class VerticalDatum { orthometric, ellipsoidal };

struct Sphereoid {
    double a;
    double b;

    Sphereoid(double a = 0, double b = 0) : a(a), b(b) {}
};

struct Srs {
    enum class Type { geographic, projected, cartesian };

    struct Modifiers { enum {
        adjustVertical = 0x01
    }; };

    std::string comment;
    Type type;
    geo::SrsDefinition srsDef;
    int srsModifiers;

    boost::optional<Sphereoid> sphereoid;
    boost::optional<VerticalDatum> vdatum;
    boost::optional<geo::SrsDefinition> srsDefEllps;

    static constexpr char typeName[] = "spatial reference system";
    typedef Dictionary<Srs> dict;

    Srs() : srsModifiers() {}

    bool adjustVertical() const {
        return srsModifiers & Modifiers::adjustVertical;
    }
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
                typedef unsigned int index_type;
                Lod lod;
                index_type x;
                index_type y;

                Id(Lod lod = 0, index_type x = 0, index_type y = 0)
                    : lod(lod), x(x), y(y)
                {}

                bool operator<(const Id &tid) const;
                bool operator==(const Id &tid) const;
            };

            struct Partitioning {
                PartitioningMode mode;

                boost::optional<math::Extents2> n00;
                boost::optional<math::Extents2> n01;
                boost::optional<math::Extents2> n10;
                boost::optional<math::Extents2> n11;
            };

            Id id;
            std::string srs;
            math::Extents2 extents;
            Partitioning partitioning;
            boost::optional<Lod> boundLayerLod;

            typedef std::map<Id, Node> map;
        };

        math::Extents3 extents;
        unsigned int rootLod;
        unsigned int arity;
        Node::map nodes;

        const Node& find(const Node::Id &id) const;
        const Node* find(const Node::Id &id, std::nothrow_t) const;

        const Node& root() const { return find({}); }

        /** Finds root node for given node id.
         */
        const Node& findSubtreeRoot(const Node::Id &nodeId) const;
    };

    std::string id;
    std::string description;
    Model model;
    Division division;

    // parameters -- generic container?
    unsigned int metaBinaryOrder;
    unsigned int navDelta;

    static constexpr char typeName[] = "reference frame";
    typedef Dictionary<ReferenceFrame> dict;

    const Division::Node& root() const { return division.root(); }

    const Division::Node& find(const Division::Node::Id &id) const {
        return division.find(id);
    }

    const Division::Node* find(const Division::Node::Id &id, std::nothrow_t)
        const
    {
        return division.find(id, std::nothrow);
    }

    const Division::Node& findSubtreeRoot(const Division::Node::Id &nodeId)
        const
    {
        return division.findSubtreeRoot(nodeId);
    }

    /** For vts0 only:
     */
    math::Extents2 rootExtents() const;
    math::Size2f tileSize(Lod lod) const;
    std::string rootSrs() const;
};

struct Position {
    enum class Type { fixed, floating, generic };

    Type type;
    math::Point3 position;
    math::Point3 orientation;
    double verticalExtent;
    double verticalFov;

    Position() : type(Type::fixed), verticalExtent(), verticalFov() {}
};

struct Credit {
    typedef CreditId NumericId;

    std::string id;
    NumericId numericId;
    std::string notice;
    boost::optional<std::string> url;
    bool copyrighted;

    Credit() : numericId(), copyrighted(false) {}

    static constexpr char typeName[] = "credit";

    typedef Dictionary<Credit> dict;
    typedef Dictionary<Credit, Credit::NumericId> ndict;
};

struct BoundLayer {
    enum Type { raster, vector };

    typedef std::uint16_t NumericId;

    std::string id;
    NumericId numericId;
    Type type;
    std::string url;
    math::Size2 tileSize;
    LodRange lodRange;
    TileRange tileRange;
    StringIdSet credits;

    BoundLayer() : numericId() {}

    static constexpr char typeName[] = "bound layer";

    typedef Dictionary<BoundLayer> dict;
    typedef Dictionary<BoundLayer, BoundLayer::NumericId> ndict;
};

// general I/O

ReferenceFrame::dict loadReferenceFrames(std::istream &in);

ReferenceFrame::dict loadReferenceFrames(const boost::filesystem::path &path);

void saveReferenceFrames(std::ostream &out
                         , const ReferenceFrame::dict &rfs);

void saveReferenceFrames(const boost::filesystem::path &path
                         , const ReferenceFrame::dict &rfs);

Srs::dict loadSrs(std::istream &in);

Srs::dict loadSrs(const boost::filesystem::path &path);

void saveSrs(std::ostream &out, const Srs::dict &srs);

void saveSrs(const boost::filesystem::path &path
             , const Srs::dict &srs);

BoundLayer::dict loadBoundLayers(std::istream &in);

BoundLayer::dict loadBoundLayers(const boost::filesystem::path &path);

void saveBoundLayers(std::ostream &out, const BoundLayer::dict &bls);

void saveBoundLayers(const boost::filesystem::path &path
                     , const BoundLayer::dict &bls);

Credit::dict loadCredits(std::istream &in);

Credit::dict loadCredits(const boost::filesystem::path &path);

void saveCredits(std::ostream &out, const Credit::dict &credits);

void saveCredits(const boost::filesystem::path &path
                 , const Credit::dict &credits);

// extra sstuff

math::Extents3 normalizedExtents(const ReferenceFrame &referenceFrame
                                 , const math::Extents3 &extents);

// enum IO stuff

UTILITY_GENERATE_ENUM_IO(VerticalDatum,
    ((orthometric))
    ((ellipsoidal))
)

UTILITY_GENERATE_ENUM_IO(PartitioningMode,
    ((bisection))
    ((manual))
    ((none))
)

UTILITY_GENERATE_ENUM_IO(Srs::Type,
    ((geographic))
    ((projected))
    ((cartesian))
)

UTILITY_GENERATE_ENUM_IO(Position::Type,
    ((fixed))
    ((floating)("float"))
    ((generic))
)

UTILITY_GENERATE_ENUM_IO(BoundLayer::Type,
    ((raster))
    ((vector))
)

// inlines

inline bool
ReferenceFrame::Division::Node::Id::operator==(const Id &id) const
{
    return ((lod == id.lod) && (x == id.x) && (y < id.y));
}

inline bool
ReferenceFrame::Division::Node::Id::operator<(const Id &id) const
{
    if (lod < id.lod) { return true; }
    else if (id.lod < lod) { return false; }

    if (x < id.x) { return true; }
    else if (id.x < x) { return false; }

    return y < id.y;
}

template<typename CharT, typename Traits>
inline std::basic_ostream<CharT, Traits>&
operator<<(std::basic_ostream<CharT, Traits> &os
           , const ReferenceFrame::Division::Node::Id &node)
{
    return os << node.lod << '-' << node.x << '-' << node.y;
}

inline math::Extents3 normalizedExtents(const ReferenceFrame &referenceFrame
                                        , const math::Extents3 &extents)
{
    // return zero extents if input not valid
    if (!valid(extents)) { return {}; }

    const auto &fe(referenceFrame.division.extents);
    const auto s(size(fe));

    return { (extents.ll(0) - fe.ll(0)) / s.width
            , (extents.ll(1) - fe.ll(1)) / s.height
            , (extents.ll(2) - fe.ll(2)) / s.depth
            , (extents.ur(0) - fe.ll(0)) / s.width
            , (extents.ur(1) - fe.ll(1)) / s.height
            , (extents.ur(2) - fe.ll(2)) / s.depth };
}

Srs::dict listSrs(const ReferenceFrame &referenceFrame);

Credit::dict creditsAsDict(const StringIdSet &credits);
Credit::dict creditsAsDict(const IdSet &credits);

BoundLayer::dict boundLayersAsDict(const StringIdSet &boundLayers);
BoundLayer::dict boundLayersAsDict(const IdSet &boundLayers);

} } // namespace vadstena::registry

#endif // vadstena_libs_registry_referenceframe_hpp_included_
