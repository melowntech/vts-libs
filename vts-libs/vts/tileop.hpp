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
#ifndef vtslibs_vts_tileop_hpp_included_
#define vtslibs_vts_tileop_hpp_included_

#include <new>

#include "math/extent.hpp"

#include "../storage/filetypes.hpp"

#include "basetypes.hpp"
#include "nodeinfo.hpp"
#include "types.hpp"

namespace vtslibs { namespace vts {

using storage::TileFile;
using storage::File;

TileId parent(const TileId &tileId, Lod diff = 1);
TileRange::point_type parent(const TileRange::point_type &point, Lod diff = 1);
TileRange parent(const TileRange &tileRange, Lod diff = 1);
TileRange tileRange(const TileId &tileId);

Children children(const TileId &tileId);
TileId lowestChild(const TileId &tileId, Lod diff = 1);
TileRange::point_type lowestChild(const TileRange::point_type &point
                                  , Lod diff = 1);
TileRange childRange(const TileRange &tileRange, Lod diff = 1);

TileRange parentRange(const TileRange &tileRange, Lod diff = 1);

/** Helper to make child range from a tile.
 *  Prerequisity: lod must be >= tileId.lod
 * \param tileId tile ID
 * \param lod exact lod
 */
TileRange childRange(const TileId &tileId, Lod lod);

TileRange shiftRange(Lod srcLod, const TileRange &tileRange, Lod dstLod);

TileRange shiftRange(const LodTileRange &tileRange, Lod dstLod);

/** Check whether super tile is above (or exactly the same tile) as tile.
 */
bool above(const TileId &tile, const TileId &super);

int child(const TileId &tileId);

bool in(const LodRange &range, const TileId &tileId);

/** Finds lowest common ancestor of two tiles defined as a lod and a range.
 */
TileId commonAncestor(Lod lod, TileRange range);

/** Calculates area of tile's mesh (m^2) and atlas (pixel^2).
 */
std::pair<double, double> area(const Tile &tile);

std::string asFilename(const TileId &tileId, TileFile type);

std::string asFilename(const TileId &tileId, TileFile type, FileFlavor flavor);

bool fromFilename(TileId &tileId, TileFile &type, unsigned int &subTileIndex
                  , const std::string &str
                  , std::string::size_type offset = 0
                  , FileFlavor *flavor = nullptr);

/** Parses "lod-x-y." or "lod-x-y-sub." from str into tileId starting at given
 *  offset. Subtile is allowed only when subfile is non-null
 *
 * Returns pointer after dot or nullptr if not found.
 */
const char *
parseTileIdPrefix(TileId &tileId, const std::string &str
                  , boost::optional<unsigned int> *subfile = nullptr
                  , std::string::size_type offset = 0);

std::string fileTemplate(TileFile type
                         , const boost::optional<unsigned int> &revision
                         = boost::none);

std::string fileTemplate(TileFile type, FileFlavor flavor
                         , const boost::optional<unsigned int> &revision
                         = boost::none);

std::string filePath(TileFile type, const TileId &tileId
                     , const boost::optional<unsigned int> &subfile
                     , const boost::optional<unsigned int> &revision
                     = boost::none);

std::size_t tileCount(Lod lod);

math::Size2f tileSize(const math::Extents2 &rootExtents, Lod lod);
double tileSize(const math::Extent &rootExtent, Lod lod);

const RFNode::Id& rfNodeId(const TileId &tileId);
const TileId& tileId(const RFNode::Id &rfNodeId);
const TileId& tileId(const NodeInfo &nodeInfo);

TileId local(Lod rootLod, const TileId &tileId);
TileId local(const NodeInfo &nodeInfo);

/** Makes global tile ID from local tile id and new root
 */
TileId global(const TileId &root, const TileId &localId);

/** Makes tile range global
 */
TileRange global(const TileId &root, Lod localLod
                 , const TileRange &localTileRange);

TileId verticalFlip(const TileId &tileId);

TileRange::point_type point(const TileId &tileId);
TileId tileId(Lod lod, const TileRange::point_type &point);
TileRange::point_type point(const NodeInfo &nodeInfo);

bool tileRangesOverlap(const TileRange &a, const TileRange &b);
TileRange tileRangesIntersect(const TileRange &a, const TileRange &b);

/** Non-throwing version of tileRangesIntersect(a, b): return invalid TileRange
 *  in case of no overlap.
 */
TileRange tileRangesIntersect(const TileRange &a, const TileRange &b
                              , const std::nothrow_t&);
math::Size2_<TileRange::value_type> tileRangesSize(const TileRange &tr);

/** Is tile withing given ranges.
 */
bool inside(const Ranges &ranges, const TileId &tileId);

/** Is tile withing given ranges or is somewhere below.
 */
bool under(const Ranges &ranges, const TileId &tileId);

/** Does given range overlap with ranges?
 */
bool overlaps(const Ranges &ranges, const LodTileRange &range);

class BorderCondition {
public:
    enum {
        top = 0x00001
        , bottom = 0x00010
        , left = 0x00100
        , right = 0x01000

        , outside = 0x00000
        , inside = 0x10000
        , flags = 0x01111
    };

    BorderCondition(int value = outside) : value_(value) {}

    operator bool() const { return value_; }

    bool check(int flags) const { return value_ & flags; }

private:
    int value_;
};

/** Determines whether tile is inside given tile range.
 *
 *  Returns:
 *      outside: 0
 *      inside: BorderingFlags::none
 *      border: bitmask composed by bitwise ORing of flags in BorderingFlags.
 *
 *  Only tiles with lod >= range.lod are considered to be inside the range
 *  unless above is true.
 *
 * NB: above functionaliy is unimplemented so far.
 */
BorderCondition inside(const LodTileRange &range, const TileId &tileId
                       , bool above = false);

/** Inflates tile extents by given margin in all 4 directions.
 *
 *  On bordering edges (defined by borderCondition) borderMargin is used
 *  instead.
 *
 *  Margins are defined as a fraction of tile width/height.
 */
math::Extents2
inflateTileExtents(const math::Extents2 &extents
                   , double margin
                   , const BorderCondition &borderCondition = BorderCondition()
                   , double borderMargin = 0);

// inline stuff

inline TileId parent(const TileId &tileId, Lod diff)
{
    // do not let new id to go above root
    if (diff > tileId.lod) { return {}; }
    return TileId(tileId.lod - diff, tileId.x >> diff, tileId.y >> diff);
}

inline TileRange::point_type parent(const TileRange::point_type &point
                                    , Lod diff)
{
    return { point(0) >> diff, point(1) >> diff };
}

inline TileRange parent(const TileRange &tileRange, Lod diff)
{
    return { parent(tileRange.ll, diff), parent(tileRange.ur, diff) };
}

inline TileRange tileRange(const TileId &tileId)
{
    return { tileId.x, tileId.y, tileId.x, tileId.y };
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

inline TileId lowestChild(const TileId &tileId, Lod diff)
{
    return TileId(tileId.lod + diff, tileId.x << diff, tileId.y << diff);
}

inline TileRange::point_type lowestChild(const TileRange::point_type &point
                                         , Lod diff)
{
    return { point(0) << diff, point(1) << diff };
}

inline TileRange childRange(const TileRange &tileRange, Lod diff)
{
    // lowest child of lower left point + original upper right point
    TileRange tr(lowestChild(tileRange.ll, diff), tileRange.ur);
    // move upper right one tile away
    ++tr.ur(0); ++tr.ur(1);

    // calculate lowest child of this point
    tr.ur = lowestChild(tr.ur, diff);

    // and move back inside original tile
    --tr.ur(0); --tr.ur(1);

    // fine
    return tr;
}

inline TileRange childRange(const TileId &tileId, Lod lod)
{
    return childRange(TileRange(point(tileId)), lod - tileId.lod);
}

inline TileRange parentRange(const TileRange &tileRange, Lod diff)
{
    auto tr(tileRange);
    tr.ll(0) >>= diff;
    tr.ll(1) >>= diff;
    tr.ur(0) >>= diff;
    tr.ur(1) >>= diff;
    return tr;
}

inline TileRange shiftRange(Lod srcLod, const TileRange &tileRange, Lod dstLod)
{
    if (srcLod == dstLod) {
        // no-op
        return tileRange;
    }

    if (dstLod > srcLod) {
        // child range
        return childRange(tileRange, dstLod - srcLod);
    }

    // parent range
    return parentRange(tileRange, srcLod - dstLod);
}

inline TileRange shiftRange(const LodTileRange &tileRange, Lod dstLod)
{
    return shiftRange(tileRange.lod, tileRange.range, dstLod);
}

inline int child(const TileId &tileId)
{
    return (tileId.x & 1l) + ((tileId.y & 1l) << 1);
}

inline bool in(const LodRange &range, const TileId &tileId)
{
    return (tileId.lod >= range.min) && (tileId.lod <= range.max);
}

inline bool in(const TileRange &range, const TileId &tileId)
{
    return ((tileId.x >= range.ll(0))
            && (tileId.x <= range.ur(0))
            && (tileId.y >= range.ll(1))
            && (tileId.y <= range.ur(1)));
}

inline std::size_t tileCount(Lod lod)
{
    return std::size_t(1) << lod;
}

inline const RFNode::Id& rfNodeId(const TileId &tileId)
{
    return tileId;
}

inline math::Size2f tileSize(const math::Extents2 &rootExtents, Lod lod)
{
    auto tc(tileCount(lod));
    auto rs(math::size(rootExtents));
    return { rs.width / tc, rs.height / tc };
}

inline double tileSize(const math::Extent &rootExtent, Lod lod)
{
    auto tc(tileCount(lod));
    auto rs(math::size(rootExtent));
    return { rs / tc };
}

inline const TileId& tileId(const RFNode::Id &rfNodeId)
{
    return rfNodeId;
}

inline const TileId& tileId(const NodeInfo &nodeInfo)
{
    return nodeInfo.nodeId();
}

inline TileId local(Lod rootLod, const TileId &tileId)
{
    if (rootLod >= tileId.lod) { return {}; }

    const auto ldiff(tileId.lod - rootLod);
    const auto mask((1 << ldiff) - 1);
    return TileId(ldiff, tileId.x & mask, tileId.y & mask);
}

inline TileId global(const TileId &root, const TileId &localId)
{
    return TileId(root.lod + localId.lod
                  , localId.x + (root.x << localId.lod)
                  , localId.y + (root.y << localId.lod));
}

inline TileRange global(const TileId &root, Lod localLod
                        , const TileRange &localTileRange)
{
    const TileRange::value_type dx(root.x << localLod);
    const TileRange::value_type dy(root.y << localLod);

    return TileRange(localTileRange.ll(0) + dx
                     , localTileRange.ll(1) + dy
                     , localTileRange.ur(0) + dx
                     , localTileRange.ur(1) + dy);
}

inline TileRange::point_type point(const TileId &tileId)
{
    return { tileId.x, tileId.y };
}

inline TileId tileId(Lod lod, const TileRange::point_type &point)
{
    return { lod, point(0), point(1) };
}

inline TileRange::point_type point(const NodeInfo &nodeInfo)
{
    return point(nodeInfo.nodeId());
}

inline bool tileRangesOverlap(const TileRange &a, const TileRange &b)
{
    // range is inclusive -> operator<=
    return ((a.ll(0) <= b.ur(0)) && (b.ll(0) <= a.ur(0))
             && (a.ll(1) <= b.ur(1)) && (b.ll(1) <= a.ur(1)));
}

inline TileRange tileRangesIntersect(const TileRange &a, const TileRange &b)
{
    if (!tileRangesOverlap(a, b)) {
        throw math::NoIntersectError
            ("Tile ranges do not overlap, cannot compute intersection");
    }

    return TileRange(TileRange::point_type
                     (std::max(a.ll[0], b.ll[0])
                      , std::max(a.ll[1], b.ll[1]))
                     , TileRange::point_type
                     (std::min(a.ur[0], b.ur[0])
                      , std::min(a.ur[1], b.ur[1])));
}

inline TileRange tileRangesIntersect(const TileRange &a, const TileRange &b
                                     , const std::nothrow_t&)
{
    if (!tileRangesOverlap(a, b)) {
        return TileRange(math::InvalidExtents{});
    }

    return TileRange(TileRange::point_type
                     (std::max(a.ll[0], b.ll[0])
                      , std::max(a.ll[1], b.ll[1]))
                     , TileRange::point_type
                     (std::min(a.ur[0], b.ur[0])
                      , std::min(a.ur[1], b.ur[1])));
}

inline math::Size2_<TileRange::value_type> tileRangesSize(const TileRange &tr)
{
    return math::Size2_<TileRange::value_type>
        (tr.ur(0) - tr.ll(0) + 1, tr.ur(1) - tr.ll(1) + 1);
}

inline TileId local(const NodeInfo &nodeInfo)
{
    return local(nodeInfo.rootLod(), nodeInfo.nodeId());
}

inline std::string fileTemplate(TileFile type
                                , const boost::optional<unsigned int>
                                &revision)
{
    return fileTemplate(type, FileFlavor::regular, revision);
}

inline std::string asFilename(const TileId &tileId, TileFile type)
{
    return asFilename(tileId, type, FileFlavor::regular);
}

inline TileId verticalFlip(const TileId &tileId)
{
    return TileId(tileId.lod, tileId.x, tileCount(tileId.lod) - 1 - tileId.y);
}

} } // namespace vtslibs::vts

#endif // vtslibs_vts_tileop_hpp_included_
