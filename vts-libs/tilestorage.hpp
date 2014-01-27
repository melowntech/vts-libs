#ifndef vadstena_libs_tilestorage_hpp_included_
#define vadstena_libs_tilestorage_hpp_included_

#include <cmath>
#include <stdexcept>
#include <string>

#include <boost/filesystem/path.hpp>

#include <opencv2/core/core.hpp>

#include "math/geometry.hpp"
#include "geometry/parse-obj.hpp"

#include "./ids.hpp"
#include "./metatile.hpp"

namespace vadstena { namespace storage {

typedef geometry::Obj Mesh;
typedef cv::Mat Atlas

/** A tile: mesh + atlas.
 */
struct Tile {
    Mesh mesh;
    Atlas atlas;
};

/** Tile identifier (index in 3D space): LOD + coordinates of lower left corner.
 */
struct TileId {
    Lod lod;
    long easting;
    long northing;
};

/** Storage properties
 */
struct Properties {
    TileId foat;    //!< Identifier of Father-of-All-Tiles metatile
    long foatSize;  //!< Size of FOAT in meters.
    Lod metaLod;    //!< Initial LOD in storage.
    Lod lodDelta;   //!< Number of LODs inside one metatile.
};

struct Error : std::runtime_error {
    Error(const std::string &message) : std::runtime_error(message) {}
};

class Storage {
public:
    /** Get tile content.
     * \param tileId idetifier of tile to return.
     * \return read tile
     * \throws Error if tile with given tileId is not found
     */
    virtual Tile getTile(const TileId &tileId) = 0;

    /** Set new tile content.
     * \param tileId idetifier of tile write.
     * \param mesh new tile's mesh
     * \param atlas new tile's atlas
     */
    virtual void setTile(const TileId &tileId, const Mesh &mesh
                         , const Atlas &atlas) = 0;

    /** Get metatile content.
     * \param tileId idetifier of metatile to return.
     * \return read metatile
     * \throws Error if metatile with given tileId is not found
     */
    virtual MetaTile getMetaTile(const TileId &tileId) = 0;

    /** Set metatile content.
     * \param tileId idetifier of metatile write.
     * \param meta new metatile's atlas
     */
    virtual void setMetaTile(const TileId &tileId, const MetaTile &meta) = 0;

    /** Query for tile's existence.
     * \param tileId identifier of queried tile
     * \return true if given tile exists, false otherwise
     *
     ** NB: Should be optimized.
     */
    virtual bool tileExists(const TileId &tileId) = 0;

    /** Get storage propeties.
     * \return storage properties
     */
    virtual Properties getProperties() = 0;

    /** Set new storage propeties.
     * \param new storage properties
     */
    virtual void setProperties(const Properties &properties) = 0;

    // convenience stuff

    /** Set new tile content.
     * \param tileId idetifier of tile write.
     * \param tile new tile's content (mesh + atlas)
     */
     */
    void setTile(const TileId &tileId, const Tile &tile);
};

inline void TileStorage::setTile(Lod lod, long easting, long northing
                                 , const Tile &tile)
{
    return setTile(lod, easting, northing, tile.mesh, tile.atlas);
}

} } // namespace vadstena::storage

#endif // vadstena_libs_tilestorage_hpp_included_
