/**
 * \file geometry_core.hpp
 * \author Vaclav Blazek <vaclav.blazek@citationtech.net>
 *
 * Tile storage manipulation.
 *
 * NB: tile set is specified by simple URI: TYPE:LOCATION where:
 *     TYPE     is type of backing storage (i.e. access driver to use);
 *              defaults to "flat"
 *     LOCATION is type-specific location of storage (e.g. root directory for
 *              filesystem based backing)
 */

#ifndef vadstena_libs_tilestorage_types_hpp_included_
#define vadstena_libs_tilestorage_types_hpp_included_

#include <memory>
#include <cmath>
#include <stdexcept>
#include <string>
#include <array>

#include <boost/filesystem/path.hpp>

#include <opencv2/core/core.hpp>

#include "math/geometry.hpp"
#include "geometry/parse-obj.hpp"

#include "../ids.hpp"
#include "../metatile.hpp"

namespace vadstena { namespace tilestorage {

typedef geometry::Obj Mesh;
typedef cv::Mat Atlas;

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

/** A tile: mesh + atlas.
 */
struct Tile {
    Mesh mesh;
    Atlas atlas;
    MetaNode metanode;
};

/** Tile identifier (index in 3D space): LOD + coordinates of lower left corner.
 */
struct TileId {
    Lod lod;
    long easting;
    long northing;

    bool operator<(const TileId &tid) const;

    TileId(Lod lod = 0, long easting = 0, long northing = 0)
        : lod(lod), easting(easting), northing(northing)
    {}
};

typedef std::array<TileId, 4> TileIdChildren;

/** Lod levels.
 */
struct LodLevels {
    Lod lod;      //!< reference lod
    Lod delta;    //!< lod step

    LodLevels() : lod(), delta() {}
};

typedef math::Point2_<long> Point2l;

typedef Point2l Alignment;

typedef math::Extents2_<long> Extents;

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

inline bool TileId::operator<(const TileId &tid) const
{
    if (lod < tid.lod) { return true; }
    else if (tid.lod < lod) { return false; }

    if (easting < tid.easting) { return true; }
    else if (tid.easting < easting) { return false; }

    return northing < tid.northing;
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

} } // namespace vadstena::tilestorage

#endif // vadstena_libs_tilestorage_types_hpp_included_
