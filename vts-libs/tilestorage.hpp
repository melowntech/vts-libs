#ifndef vadstena_libs_tilestorage_hpp_included_
#define vadstena_libs_tilestorage_hpp_included_

#include <memory>
#include <cmath>
#include <stdexcept>
#include <string>

#include <boost/filesystem/path.hpp>

#include <opencv2/core/core.hpp>

#include "math/geometry.hpp"
#include "geometry/parse-obj.hpp"

#include "./ids.hpp"
#include "./metatile.hpp"

namespace vadstena { namespace tilestorage {

typedef geometry::Obj Mesh;
typedef cv::Mat Atlas;

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

    bool operator<(const TileId &tid) const;
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
    /** Pointer type.
     */
    typedef std::shared_ptr<Storage> pointer;

    virtual ~Storage();

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

    /** Get tile's metadata.
     * \param tileId idetifier of metatile to return.
     * \return read metadata
     * \throws Error if metadate with given tileId is not found
     */
    virtual MetaNode getMetaData(const TileId &tileId) = 0;

    /** Set tile's metadata.
     * \param tileId idetifier of tile to write metadata to.
     * \param meta new tile's metadata
     */
    virtual void setMetaData(const TileId &tileId, const MetaNode &meta) = 0;

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

    /** Flush all pending changes to permanent storage.
     *
     * If storage is not allowed to be changed (i.e. open in read-only mode)
     * flush call is ignored.
     */
    virtual void flush() = 0;

    // convenience stuff

    /** Set new tile content.
     * \param tileId idetifier of tile write.
     * \param tile new tile's content (mesh + atlas)
     */
    void setTile(const TileId &tileId, const Tile &tile);

    /** Needed to instantiate subclasses.
     */
    class Factory;
};

/** Open mode
 */
enum class OpenMode {
    readOnly     //!< only getters are allowed
    , readWrite  //!< both getters and setters are allowed
};

enum class CreateMode {
    failIfExists //!< creation fails if storage already exists
    , overwrite  //!< existing storage is replace with new one
};

/** Creates new storage.
 *
 * \param uri URI that specifies storage type and location.
 * \param properties properties to initialize new storage with
 * \param mode what to do when storage already exists:
 *                 * failIfExists: storage must not exists prior this call
 *                 * overwrite: new storage is created
 * \return interface to new storage
 * \throws Error if storage cannot be created
 */
Storage::pointer create(const std::string &uri
                        , const Properties &properties
                        , CreateMode mode = CreateMode::failIfExists);

/** Opens existing storage.
 *
 * \param uri URI that specifies storage type and location.
 * \param properties properties to initialize new storage with
 * \param mode what operations are allowed on storage:
 *                 * readOnly: only getters are allowed
 *                 * readWrite: both getters and setters are allowed
 * \return interface to new storage
 * \throws Error if storage cannot be opened
 */
Storage::pointer open(const std::string &uri
                      , OpenMode mode = OpenMode::readOnly);



// inline stuff

inline void Storage::setTile(const TileId &tileId, const Tile &tile)
{
    return setTile(tileId, tile.mesh, tile.atlas);
}

inline bool TileId::operator<(const TileId &tid) const
{
    if (lod < tid.lod) { return true; }
    else if (tid.lod < lod) { return false; }

    if (easting < tid.easting) { return true; }
    else if (tid.easting < easting) { return false; }

    return northing < tid.northing;
}

} } // namespace vadstena::tilestorage

#endif // vadstena_libs_tilestorage_hpp_included_
