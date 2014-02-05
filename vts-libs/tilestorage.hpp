#ifndef vadstena_libs_tilestorage_hpp_included_
#define vadstena_libs_tilestorage_hpp_included_

#include <memory>
#include <cmath>
#include <stdexcept>
#include <string>
#include <array>

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

/** Storage properties that must be specified during creation. They cannot be
 *  changed later.
 */
struct CreateProperties {
    /** Unique set identifier.
     */
    std::string id;

    /** Metatile lod levels (metaLevels.lod + n * metaLevels.delta).
     */
    LodLevels metaLevels;

    /** Tile size at LOD=0.
     */
    long baseTileSize;

    /** Tile alignment. No tile exists that contains this point inside.
     */
    Alignment alignment;

    CreateProperties() : baseTileSize() {}
};

/** Storage properties that can be set anytime.
 */
struct SettableProperties {
    math::Point3 defaultPosition;    // easting, northing, altitude
    math::Point3 defaultOrientation; // yaw, pitch, roll
    short textureQuality;            // JPEG quality

    struct Mask { enum {             // mask bitfields
        defaultPosition = 0x01
        , defaultOrientation = 0x02
        , textureQuality = 0x04
    }; };

    SettableProperties()
        : defaultOrientation(0, -90, 0)
        , textureQuality(85) {}

    bool merge(const SettableProperties &other, int mask = ~0);
};

/** All storage properties.
 */
struct Properties
    : CreateProperties
    , SettableProperties
{
    TileId foat;    //!< Identifier of Father-of-All-Tiles metatile
    long foatSize;  //!< Size of FOAT in meters.

    std::string meshTemplate;     //!< mesh file template
    std::string textureTemplate;  //!< texture file template
    std::string metaTemplate;     //!< meta tile file template

    Properties() : foatSize() {}
};

struct Error : std::runtime_error {
    Error(const std::string &message) : std::runtime_error(message) {}
};

struct NoSuchTileSet : Error {
    NoSuchTileSet(const std::string &message) : Error(message) {}
};

struct TileSetAlreadyExists : Error {
    TileSetAlreadyExists(const std::string &message) : Error(message) {}
};

struct FormatError : Error {
    FormatError(const std::string &message) : Error(message) {}
};

struct NoSuchTile : Error {
    NoSuchTile(const std::string &message) : Error(message) {}
};

/** Driver that implements physical aspects of tile storage.
 */
class Driver;

/** TileSet interface.
 */
class TileSet {
public:
    /** Pointer type.
     */
    typedef std::shared_ptr<TileSet> pointer;

    ~TileSet();

    /** Get tile content.
     * \param tileId idetifier of tile to return.
     * \return read tile
     * \throws Error if tile with given tileId is not found
     */
    Tile getTile(const TileId &tileId) const;

    /** Set new tile content.
     * \param tileId idetifier of tile write.
     * \param mesh new tile's mesh
     * \param atlas new tile's atlas
     * \param metadata initial metadata of tile (optional)
     */
    void setTile(const TileId &tileId, const Mesh &mesh, const Atlas &atlas
                 , const TileMetadata *metadata = nullptr);

    /** Set new tile's metadata.
     * \param tileId idetifier of tile write
     * \param metadata new tile metadata
     * \throw Error if tile doesn't exist
     */
    void setMetadata(const TileId &tileId, const TileMetadata &metadata);

    /** Query for tile's existence.
     * \param tileId identifier of queried tile
     * \return true if given tile exists, false otherwise
     *
     ** NB: Should be optimized.
     */
    bool tileExists(const TileId &tileId) const;

    /** Get storage propeties.
     * \return storage properties
     */
    Properties getProperties() const;

    /** Set new storage propeties.
     * \param properties new storage properties
     * \param mask bitmask marking fields to be udpated
     *             (defaults to all fields); use SettableProperties::Mask
     * \return all new storage properties after change
     */
    Properties setProperties(const SettableProperties &properties
                             , int mask = ~0);

    /** Flush all pending changes to permanent storage.
     *
     * If storage is not allowed to be changed (i.e. open in read-only mode)
     * flush call is ignored.
     */
    void flush();

    /** Needed to instantiate subclasses.
     */
    class Factory;
    friend class Factory;

private:
    TileSet(const std::shared_ptr<Driver> &driver);

    struct Detail;
    std::unique_ptr<Detail> detail_;
    Detail& detail() { return *detail_; }
    const Detail& detail() const { return *detail_; }
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

/** Creates new tile set.
 *
 * \param uri URI that specifies tile set type and location.
 * \param properties properties to initialize new tile set with
 * \param mode what to do when tile set already exists:
 *                 * failIfExists: tile set must not exists prior this call
 *                 * overwrite: new tile set is created
 * \return interface to new tile set
 * \throws Error if tile set cannot be created
 */
TileSet::pointer createTileSet(const std::string &uri
                               , const CreateProperties &properties
                               , CreateMode mode = CreateMode::failIfExists);

/** Opens existing tile set.
 *
 * \param uri URI that specifies tile set type and location.
 * \param mode what operations are allowed on tile set:
 *                 * readOnly: only getters are allowed
 *                 * readWrite: both getters and setters are allowed
 * \return interface to new tile set
 * \throws Error if tile set cannot be opened
 */
TileSet::pointer openTileSet(const std::string &uri
                             , OpenMode mode = OpenMode::readOnly);


// inline stuff

inline bool TileId::operator<(const TileId &tid) const
{
    if (lod < tid.lod) { return true; }
    else if (tid.lod < lod) { return false; }

    if (easting < tid.easting) { return true; }
    else if (tid.easting < easting) { return false; }

    return northing < tid.northing;
}

inline bool SettableProperties::merge(const SettableProperties &other
                                      , int mask)
{
    bool changed(false);
#define SETTABLEPROPERTIES_MERGE(WHAT) \
    if (mask & Mask::WHAT) { WHAT = other.WHAT; changed = true; }

    SETTABLEPROPERTIES_MERGE(defaultPosition);
    SETTABLEPROPERTIES_MERGE(defaultOrientation);
    SETTABLEPROPERTIES_MERGE(textureQuality);

#undef SETTABLEPROPERTIES_MERGE
    return changed;
}

} } // namespace vadstena::tilestorage

#endif // vadstena_libs_tilestorage_hpp_included_
