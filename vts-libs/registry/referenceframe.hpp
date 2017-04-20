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
/**
 * \file registry/referenceframe.hpp
 * \author Vaclav Blazek <vaclav.blazek@citationtech.net>
 */

#ifndef vtslibs_registry_referenceframe_hpp_included_
#define vtslibs_registry_referenceframe_hpp_included_

#include <set>
#include <map>
#include <string>
#include <vector>
#include <new>

#include "dbglog/dbglog.hpp"

#include "utility/enum-io.hpp"
#include "utility/streams.hpp"

#include "geo/srsdef.hpp"

#include "../storage/error.hpp"
#include "../storage/lod.hpp"
#include "../storage/credits.hpp"

#include "./types.hpp"
#include "./dict.hpp"

namespace vtslibs { namespace registry {

enum class PartitioningMode {
    /** children divide this sds node's extents in half in both dimensions
     */
    bisection

    /** children start new division
     */
    , manual

    /** invalid sds node
     */
    , none

    /** unproductive sds node, exists solely to complete tree from root
     */
    , barren
};

struct GeoidGrid {
    math::Extents2 extents;
    HeightRange valueRange;
    std::string definition;
    geo::SrsDefinition srsDefEllps;
};

struct Periodicity {
    enum class Type { x, y };

    Type type;
    double period;

    Periodicity() : type(), period() {}
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

    boost::optional<GeoidGrid> geoidGrid;
    boost::optional<Periodicity> periodicity;

    static constexpr char typeName[] = "spatial reference system";
    typedef StringDictionary<Srs> dict;

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
                bool operator!=(const Id &tid) const;

                typedef std::vector<Id> list;
            };

            struct Partitioning {
                PartitioningMode mode;

                boost::optional<math::Extents2> n00;
                boost::optional<math::Extents2> n01;
                boost::optional<math::Extents2> n10;
                boost::optional<math::Extents2> n11;

                Partitioning(PartitioningMode mode
                             = PartitioningMode::bisection)
                    : mode(mode) {}
            };

            /** Constraints on a node.
             */
            struct Constraints {
                /** Extra extents to apply.
                 */
                math::Extents2 extents;

                /** SRS of extra constraints
                 */
                std::string extentsSrs;

                Constraints(const math::Extents2 &extents
                            , const std::string &extentsSrs)
                    : extents(extents), extentsSrs(extentsSrs)
                {}
            };


            /** Structure navigation.
             */
            struct Structure {
                Id parent;
                typedef std::uint8_t ChildFlags;
                enum : ChildFlags {
                    c00  = 0x01
                   , c10 = 0x02
                   , c01 = 0x04
                   , c11 = 0x08
                };
                ChildFlags children;

                Structure() : children() {}
            };

            Id id;
            std::string srs;
            math::Extents2 extents;
            Partitioning partitioning;
            bool externalTexture;

            /** Extra constraints applied on this node. Copied from parent node
             *  in case of manual partitioning.
             */
            boost::optional<Constraints> constraints;

            /** Navigation through structure.
             */
            Structure structure;

            typedef std::map<Id, Node> map;

            Node() : externalTexture(false) {}
            Node(const Id &id) : id(id), externalTexture(false) {}
            Node(const Id &id, PartitioningMode mode)
                : id(id), partitioning(mode), externalTexture(false) {}

            bool valid() const {
                return (partitioning.mode != PartitioningMode::none);
            }

            /** Real node is a node whose subtree can contain tile. Invalid and
             *  barren nodes are not real.
             */
            bool real() const {
                switch (partitioning.mode) {
                case PartitioningMode::manual:
                case PartitioningMode::bisection:
                    return true;
                default: return false;
                }
            }

            void invalidate() {
                partitioning.mode = PartitioningMode::none;
            }
        };

        math::Extents3 extents;
        HeightRange heightRange;
        Node::map nodes;

        /** [synthetic] Union of LODs of all nodes defined in the reference
         *  frame. To be used for better tree navigation. Only nodes in this
         *  range can be subtree roots
         */
        LodRange rootLodRange;

        const Node& find(const Node::Id &id) const;
        const Node* find(const Node::Id &id, std::nothrow_t) const;

        Node* find(const Node::Id &id, std::nothrow_t);

        const Node& root() const { return find({}); }

        /** Finds root node for given node id.
         */
        const Node& findSubtreeRoot(const Node::Id &nodeId) const;

        /** Finds root node for given node id. Doesn't throw.
         */
        const Node* findSubtreeRoot(Node::Id nodeId, std::nothrow_t)
            const;

        /** Get list of all SRS's used by this division
         */
        std::set<std::string> srsList() const;

        Division() : rootLodRange(LodRange::emptyRange()) {}
    };

    std::string id;
    std::string description;
    Model model;
    Division division;

    // parameters -- generic container?
    unsigned int metaBinaryOrder;

    static constexpr char typeName[] = "reference frame";
    typedef StringDictionary<ReferenceFrame> dict;

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

    const Division::Node* findSubtreeRoot(const Division::Node::Id &nodeId
                                          , std::nothrow_t)
        const
    {
        return division.findSubtreeRoot(nodeId, std::nothrow);
    }

    void invalidate(const Division::Node::Id &nodeId);

    /** For vts0 only:
     */
    math::Extents2 rootExtents() const;
    math::Size2f tileSize(Lod lod) const;
    std::string rootSrs() const;

    ReferenceFrame() : metaBinaryOrder(5) {}
};

struct Position {
    enum class Type { objective, subjective };
    enum class HeightMode { fixed, floating };

    Type type;
    HeightMode heightMode;
    math::Point3 position;
    math::Point3 orientation;
    double verticalExtent;
    double verticalFov;

    Position()
        : type(Type::objective), heightMode(HeightMode::fixed)
        , verticalExtent(), verticalFov()
    {}

    bool valid() { return orientation != math::Point3(0, 0, 0); }

    bool operator==(const Position &p) const;
    bool operator!=(const Position &p) const { return !operator==(p); }

    void lookDown() { orientation = { .0, -90., .0 }; }
    static double naturalFov() { return 55; }
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

    typedef DualDictionary<Credit, Credit::NumericId> dict;
};

typedef StringDictionary<boost::optional<Credit>> Credits;

struct BoundLayer {
    static constexpr unsigned int metaBinaryOrder = 8;

    enum class Type { raster, vector, external };

    typedef std::uint16_t NumericId;

    std::string id;
    NumericId numericId;

    Type type;
    std::string url;
    boost::optional<std::string> maskUrl;
    boost::optional<std::string> metaUrl;
    boost::optional<std::string> creditsUrl;
    LodRange lodRange;
    TileRange tileRange;

    /** Credits make sense only when creditsUrl is invalid
     */
    Credits credits;

    struct Availability {
        enum Type { negativeType, negativeCode, negativeSize };
        Type type;

        std::string mime;
        std::set<int> codes;
        int size;
    };

    boost::optional<Availability> availability;
    bool isTransparent;

    BoundLayer() : numericId(), isTransparent(false) {}

    explicit BoundLayer(const std::string &id, const std::string &url)
        : id(id), numericId(), type(Type::external)
        , url(url), isTransparent(false)
    {}

    bool external() const { return type == Type::external; }

    static constexpr char typeName[] = "bound layer";

    static constexpr int tileWidth = 256;
    static constexpr int tileHeight = 256;
    static constexpr int basicTileArea = (tileWidth * tileHeight);

    static math::Size2 tileSize() { return { tileWidth, tileHeight }; }
    static double tileArea() { return basicTileArea; }
    static math::Size2 metaSize() { return { tileWidth, tileHeight }; }
    static double metaArea() { return basicTileArea; }
    static math::Size2 maskSize() { return { tileWidth, tileHeight }; }
    static double maskArea() { return basicTileArea; }

    // boundlayer metatile constants

    static constexpr int rasterMetatileBinaryOrder = 8;
    static constexpr int rasterMetatileWidth = 1 << rasterMetatileBinaryOrder;
    static constexpr int rasterMetatileHeight = 1 << rasterMetatileBinaryOrder;
    static math::Size2 rasterMetatileSize() {
        return { rasterMetatileBinaryOrder, rasterMetatileBinaryOrder};
    }

    struct MetaFlags {
        static constexpr std::uint8_t watertight = 0xc0;
        static constexpr std::uint8_t available = 0x80;
        static constexpr std::uint8_t unavailable = 0x00;
    };

    typedef DualDictionary<BoundLayer, BoundLayer::NumericId> dict;
};

// general I/O

ReferenceFrame::dict loadReferenceFrames(std::istream &in
                                         , const boost::filesystem::path &path
                                         = "UNKNOWN");

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

void saveBoundLayer(std::ostream &out, const BoundLayer &bl);

BoundLayer loadBoundLayer(std::istream &in
                          , const boost::filesystem::path &path
                          = "unknown");

Credit::dict loadCredits(std::istream &in
                         , const boost::filesystem::path &path
                         = "unknown");

Credit::dict loadCredits(const boost::filesystem::path &path);

void saveCredits(std::ostream &out, const Credit::dict &credits);

void saveCredits(const boost::filesystem::path &path
                 , const Credit::dict &credits);

void loadCredits(std::istream &in, Credits &credits
                 , const boost::filesystem::path &path
                 = "unknown");

void saveCredits(std::ostream &out, const Credits &credits
                 , bool inlineCredits = true);

// extra stuff

math::Extents3 normalizedExtents(const ReferenceFrame &referenceFrame
                                 , const math::Extents3 &extents);

// enum IO stuff

UTILITY_GENERATE_ENUM_IO(PartitioningMode,
    ((bisection))
    ((manual))
    ((none)("#none"))
    ((barren)("#barren"))
)

UTILITY_GENERATE_ENUM_IO(Periodicity::Type,
    ((x)("X"))
    ((y)("Y"))
)

UTILITY_GENERATE_ENUM_IO(Srs::Type,
    ((geographic))
    ((projected))
    ((cartesian))
)

UTILITY_GENERATE_ENUM_IO(Position::Type,
    ((objective)("obj"))
    ((subjective)("subj"))
)

UTILITY_GENERATE_ENUM_IO(Position::HeightMode,
    ((fixed)("fixed")("fix"))
    ((floating)("float"))
)

UTILITY_GENERATE_ENUM_IO(BoundLayer::Type,
    ((raster))
    ((vector))
    ((external))
)

UTILITY_GENERATE_ENUM_IO(BoundLayer::Availability::Type,
    ((negativeType)("negative-type"))
    ((negativeCode)("negative-code"))
    ((negativeSize)("negative-size"))
)

// inlines

inline bool ReferenceFrame::Division::Node::Id::operator==(const Id &id) const
{
    return ((lod == id.lod) && (x == id.x) && (y == id.y));
}

inline bool ReferenceFrame::Division::Node::Id::operator!=(const Id &id) const
{
    return ((lod != id.lod) || (x != id.x) ||(y != id.y));
}

inline bool ReferenceFrame::Division::Node::Id::operator<(const Id &id) const
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

template<typename CharT, typename Traits>
inline std::basic_istream<CharT, Traits>&
operator>>(std::basic_istream<CharT, Traits> &is
           , ReferenceFrame::Division::Node::Id &node)
{
    return is >> node.lod
              >> utility::expect('-') >> node.x
              >> utility::expect('-') >> node.y;
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

inline bool Position::operator==(const Position &p) const
{
    return ((type == p.type)
            && (heightMode == p.heightMode)
            && (position == p.position)
            && (orientation == p.orientation)
            && (verticalExtent == p.verticalExtent)
            && (verticalFov == p.verticalFov));
}

Srs::dict listSrs(const ReferenceFrame &referenceFrame);

Credit::dict creditsAsDict(const StringIdSet &credits);
Credit::dict creditsAsDict(const Credits &credits);
Credit::dict creditsAsDict(const IdSet &credits);

BoundLayer::dict boundLayersAsDict(const StringIdSet &boundLayers);
BoundLayer::dict boundLayersAsDict(const IdSet &boundLayers);

} } // namespace vtslibs::registry

#endif // vtslibs_registry_referenceframe_hpp_included_
