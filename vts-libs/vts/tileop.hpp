#ifndef vadstena_libs_vts_tileop_hpp_included_
#define vadstena_libs_vts_tileop_hpp_included_

#include "../storage/filetypes.hpp"

#include "./basetypes.hpp"
#include "./types.hpp"

namespace vadstena { namespace vts {

using storage::TileFile;
using storage::File;

TileId parent(const TileId &tileId, Lod diff = 1);

Children children(const TileId &tileId);

/** Check whether super tile is above (or exactly the same tile) as tile.
 */
bool above(const TileId &tile, const TileId &super);

int child(const TileId &tileId);

bool in(const LodRange &range, const TileId &tileId);

/** Calculates area of tile's mesh (m^2) and atlas (pixel^2).
 */
std::pair<double, double> area(const Tile &tile);

std::string asFilename(const TileId &tileId, TileFile type);

bool fromFilename(TileId &tileId, TileFile &type, unsigned int &subTileIndex
                  , const std::string &str
                  , std::string::size_type offset = 0);

std::string fileTemplate(TileFile type
                         , boost::optional<unsigned int> revision
                         = boost::none);

std::size_t tileCount(Lod lod);

math::Size2f tileSize(const math::Extents2 &rootExtents, Lod lod);

RFNode::Id rfNodeId(const TileId &tileId);
TileId tileId(const RFNode::Id &rfNodeId);
TileId tileId(const NodeInfo &nodeInfo);

TileId local(Lod rootLod, const TileId &tileId);

math::Point2_<unsigned int> point(const TileId &tileId);
TileId tileId(Lod lod, const math::Point2_<unsigned int> &point);
math::Point2_<unsigned int> point(const NodeInfo &nodeInfo);

// inline stuff

inline TileId parent(const TileId &tileId, Lod diff)
{
    return TileId(tileId.lod - diff, tileId.x >> diff, tileId.y >> diff);
}

inline Children children(const TileId &tileId)
{
    TileId base(tileId.lod + 1, tileId.x << 1, tileId.y << 1);

    return {{
        { base, 0 }                                  // upper-left
        , { base.lod, base.x + 1, base.y, 1 }      // upper-right
        , { base.lod, base.x, base.y + 1, 2 }        // lower-left
        , { base.lod, base.x + 1, base.y + 1, 3}  // lower-right
    }};
}

inline int child(const TileId &tileId)
{
    return (tileId.x & 1l) + ((tileId.y & 1l) << 1);
}

inline bool in(const LodRange &range, const TileId &tileId)
{
    return (tileId.lod >= range.min) && (tileId.lod <= range.max);
}

inline std::size_t tileCount(Lod lod)
{
    return std::size_t(1) << lod;
}

inline RFNode::Id rfNodeId(const TileId &tileId)
{
    return RFNode::Id
        (tileId.lod, tileId.x, tileId.y);
}

inline math::Size2f tileSize(const math::Extents2 &rootExtents, Lod lod)
{
    auto tc(tileCount(lod));
    auto rs(math::size(rootExtents));
    return { rs.width / tc, rs.height / tc };
}

inline TileId tileId(const RFNode::Id &rfNodeId)
{
    return TileId(rfNodeId.lod, rfNodeId.x, rfNodeId.y);
}

inline TileId tileId(const NodeInfo &nodeInfo)
{
    return tileId(nodeInfo.nodeId());
}

inline TileId local(Lod rootLod, const TileId &tileId)
{
    if (rootLod >= tileId.lod) { return {}; }

    const auto ldiff(tileId.lod - rootLod);
    const auto mask((1 << ldiff) - 1);
    return TileId(ldiff, tileId.x & mask, tileId.y & mask);
}

inline math::Point2_<unsigned int> point(const TileId &tileId)
{
    return { tileId.x, tileId.y };
}

inline TileId tileId(Lod lod, const math::Point2_<unsigned int> &point)
{
    return { lod, point(0), point(1) };
}

inline math::Point2_<unsigned int> point(const NodeInfo &nodeInfo)
{
    return { nodeInfo.node.id.x, nodeInfo.node.id.y };
}

} } // namespace vadstena::vts

#endif // vadstena_libs_vts_tileop_hpp_included_
