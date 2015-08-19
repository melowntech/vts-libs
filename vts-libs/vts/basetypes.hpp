/**
 * \file basertypes.hpp
 * \author Vaclav Blazek <vaclav.blazek@citationtech.net>
 *
 * TileStorage base types.
 *
 * NB: tile set is specified by simple URI: TYPE:LOCATION where:
 *     TYPE     is type of backing storage (i.e. access driver to use);
 *              defaults to "flat"
 *     LOCATION is type-specific location of storage (e.g. root directory for
 *              filesystem based backing)
 */

#ifndef vadstena_libs_vts_basetypes_hpp_included_
#define vadstena_libs_vts_basetypes_hpp_included_

#include <string>

#include "math/geometry_core.hpp"

#include "../ids.hpp"

namespace vadstena { namespace vts {

/** Tile set locator
 */
struct Locator {
    std::string type;     // tile set type (i.e. driver to use)
    std::string location; // location of tile set

    /** Parse locator from TYPE:LOCATION
     */
    Locator(const std::string &locator);

    Locator(const std::string &type, const std::string &location)
        : type(type), location(location)
    {}

    Locator() {}

    std::string asString() const;
};

/** Tile identifier (index in 3D space): LOD + coordinates of lower left corner.
 */
struct TileId {
    Lod lod;
    long x;
    long y;

    bool operator<(const TileId &tid) const;
    bool operator==(const TileId &tid) const;
    bool operator!=(const TileId &tid) const { return !operator==(tid); }

    TileId(Lod lod = 0, long x = 0, long y = 0)
        : lod(lod), x(x), y(y)
    {}
};

typedef math::Size2_<long> Size2l;

typedef math::Point2_<long> Point2l;
typedef std::vector<Point2l> Points2l;

typedef Point2l Alignment;

typedef math::Extents2_<long> Extents;

typedef std::array<TileId, 4> Children;

/** Lod levels.
 */
struct LodLevels {
    Lod lod;      //!< reference lod
    Lod delta;    //!< lod step

    LodLevels() : lod(), delta() {}
};

/** Open mode
 */
enum class OpenMode {
    readOnly     //!< only getters are allowed
    , readWrite  //!< both getters and setters are allowed
};

enum class CreateMode {
    failIfExists //!< creation fails if tile set/storage already exists
    , overwrite  //!< existing tile set/storage is replace with new one
};

// inline stuff

inline bool TileId::operator<(const TileId &tileId) const
{
    if (lod < tileId.lod) { return true; }
    else if (tileId.lod < lod) { return false; }

    if (x < tileId.x) { return true; }
    else if (tileId.x < x) { return false; }

    return y < tileId.y;
}

inline bool TileId::operator==(const TileId &tid) const
{
    return ((lod == tid.lod)
            && (x == tid.x)
            && (y == tid.y));
}

inline Locator::Locator(const std::string &locator)
{
    auto idx(locator.find(':'));
    if (idx == std::string::npos) {
        type = {};
        location = locator;
    } else {
        type = locator.substr(0, idx);
        location = locator.substr(idx + 1);
    }
}

inline std::string Locator::asString() const
{
    std::string s(type);
    s.push_back(':');
    s.append(location);
    return s;
}

inline bool operator==(const LodLevels &l, const LodLevels &r)
{
    // deltas must be same
    if (l.delta != r.delta) { return false; }
    // difference between reference lods must be divisible by delta
    if (std::abs(l.lod - r.lod) % l.delta) { return false; }
    return true;
}

inline bool operator!=(const LodLevels &l, const LodLevels &r)
{
    return !operator==(l, r);
}

inline Point2l point(const TileId &tid)
{
    return { tid.x, tid.y };
}

} } // namespace vadstena::vts

#endif // vadstena_libs_vts_basetypes_hpp_included_
