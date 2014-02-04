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

struct NoSuchStorage : Error {
    NoSuchStorage(const std::string &message) : Error(message) {}
};

struct StorageAlreadyExists : Error {
    StorageAlreadyExists(const std::string &message) : Error(message) {}
};

struct FormatError : Error {
    FormatError(const std::string &message) : Error(message) {}
};

struct NoSuchTile : Error {
    NoSuchTile(const std::string &message) : Error(message) {}
};

/** Storage interface.
 *
 * This class is pure virtual class. To create instance of storage use create()
 * or open() free standing function.
 *
 * NB: this class follows Non-Virtual Interface Idiom defined by Herb Sutter in
 * his "Virtuality" article at <http://www.gotw.ca/publications/mill18.htm>.
 *
 * All you need to write your own storage is to inherit this class and implement
 * all* _impl member functions. Also, you need to register your class inside
 * create() and open() functions.
 */
class Storage {
public:
    /** Pointer type.
     */
    typedef std::shared_ptr<Storage> pointer;

    virtual ~Storage() = 0;

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

private:
    /** Override in derived class. Called from getTile.
     */
    virtual Tile getTile_impl(const TileId &tileId) const = 0;

    /** Override in derived class. Called from setTile.
     */
    virtual void setTile_impl(const TileId &tileId, const Mesh &mesh
                              , const Atlas &atlas
                              , const TileMetadata *metadata) = 0;

    /** Override in derived class. Called from setMetadata.
     */
    virtual void setMetadata_impl(const TileId &tileId
                                  , const TileMetadata &metadata) = 0;

    /** Override in derived class. Called from tileExists.
     */
    virtual bool tileExists_impl(const TileId &tileId) const = 0;

    /** Override in derived class. Called from getProperties.
     */
    virtual Properties getProperties_impl() const = 0;

    /** Override in derived class. Called from setProperties.
     */
    virtual Properties setProperties_impl(const SettableProperties &properties
                                          , int mask) = 0;

    /** Override in derived class. Called from flush.
     */
    virtual void flush_impl() = 0;

public:
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
                        , const CreateProperties &properties
                        , CreateMode mode = CreateMode::failIfExists);

/** Opens existing storage.
 *
 * \param uri URI that specifies storage type and location.
 * \param mode what operations are allowed on storage:
 *                 * readOnly: only getters are allowed
 *                 * readWrite: both getters and setters are allowed
 * \return interface to new storage
 * \throws Error if storage cannot be opened
 */
Storage::pointer open(const std::string &uri
                      , OpenMode mode = OpenMode::readOnly);


long tileSize(long baseTileSize, Lod lod);

long tileSize(const Properties &properties, Lod lod);

Extents tileExtents(const Properties &properties, const TileId &tile);

TileId fromAlignment(const Properties &properties, const TileId &tileId);

TileId parent(const Properties &properties, const TileId &tileId);

TileIdChildren children(long baseTileSize, const TileId &tileId);

// inline stuff

inline Tile Storage::getTile(const TileId &tileId) const
{
    return getTile_impl(tileId);
}

inline void Storage::setTile(const TileId &tileId, const Mesh &mesh
                             , const Atlas &atlas
                             , const TileMetadata *metadata)
{
    return setTile_impl(tileId, mesh, atlas, metadata);
}

inline void Storage::setMetadata(const TileId &tileId
                                 , const TileMetadata &metadata)
{
    return setMetadata_impl(tileId, metadata);
}

inline bool Storage::tileExists(const TileId &tileId) const
{
    return tileExists_impl(tileId);
}

inline Properties Storage::getProperties() const
{
    return getProperties_impl();
}

inline Properties Storage::setProperties(const SettableProperties &properties
                                         , int mask)
{
    return setProperties_impl(properties, mask);
}

inline void Storage::flush()
{
    return flush_impl();
}

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

inline long tileSize(long baseTileSize, Lod lod)
{
    return (baseTileSize >> lod);
}

inline long tileSize(const Properties &properties, Lod lod)
{
    return (properties.baseTileSize >> lod);
}

inline Extents tileExtents(const Properties &properties, const TileId &tile)
{
    auto ts(tileSize(properties, tile.lod));
    return { tile.easting, tile.northing
            , tile.easting + ts, tile.northing + ts };
}

inline TileId fromAlignment(const Properties &properties, const TileId &tileId)
{
    return { tileId.lod, tileId.easting - properties.alignment(0)
            , tileId.northing - properties.alignment(1) };
}

inline TileId parent(const Properties &properties, const TileId &tileId)
{
    auto aligned(fromAlignment(properties, tileId));
    auto ts(tileSize(properties, tileId.lod));
    aligned.easting /= ts;
    aligned.northing /= ts;

    constexpr auto mask(~(static_cast<decltype(tileId.easting)>(1)));

    return {
        Lod(tileId.lod - 1)
        , properties.alignment(0) + (aligned.easting & mask) * ts
        , properties.alignment(1) + (aligned.northing & mask) * ts
    };
}

inline TileIdChildren children(long baseTileSize, const TileId &tileId)
{
    auto ts(tileSize(baseTileSize, tileId.lod));

    Lod lod(tileId.lod -1);

    return {{
        { lod, tileId.easting, tileId.northing }               // lower-left
        , { lod, tileId.easting, tileId.northing + ts }        // lower-right
        , { lod, tileId.easting + ts, tileId.northing + ts }   // upper-left
        , { lod, tileId.easting + ts, tileId.northing }        // upper-right
    }};
}

} } // namespace vadstena::tilestorage

#endif // vadstena_libs_tilestorage_hpp_included_
