/**
 * \file tilestorage/tileset.hpp
 * \author Vaclav Blazek <vaclav.blazek@citationtech.net>
 *
 * Tile set access.
 *
 * NB: tile set is specified by simple URI: TYPE:LOCATION where:
 *     TYPE     is type of backing storage (i.e. access driver to use);
 *              defaults to "flat"
 *     LOCATION is type-specific location of storage (e.g. root directory for
 *              filesystem based backing)
 */

#ifndef vadstena_libs_tilestorage_tileset_hpp_included_
#define vadstena_libs_tilestorage_tileset_hpp_included_

#include <memory>
#include <cmath>
#include <list>
#include <string>
#include <array>

#include <boost/noncopyable.hpp>
#include <boost/filesystem/path.hpp>

#include "./types.hpp"
#include "../ids.hpp"
#include "../range.hpp"

namespace vadstena { namespace tilestorage {

/** Driver that implements physical aspects of tile set.
 */
class Driver;

/** Tile set properties that must be specified during creation. They cannot be
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

/** Tile set properties that can be set anytime.
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

/** All tile set properties.
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

/** TileSet interface.
 */
class TileSet : boost::noncopyable {
public:
    /** Pointer type.
     */
    typedef std::shared_ptr<TileSet> pointer;

    typedef std::vector<pointer> list;

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

    /** Get tile set propeties.
     * \return tile set properties
     */
    Properties getProperties() const;

    /** Set new tile set propeties.
     * \param properties new tile set properties
     * \param mask bitmask marking fields to be udpated
     *             (defaults to all fields); use SettableProperties::Mask
     * \return all new tile set properties after change
     */
    Properties setProperties(const SettableProperties &properties
                             , int mask = ~0);

    /** Flush all pending changes to backing store (e.g. filesystem).
     *
     * If tile set is not allowed to be changed (i.e. open in read-only mode)
     * flush call is ignored.
     */
    void flush();

    /** Starts new transaction. (R/W)
     */
    void begin();

    /** Commits pending transaction. Calls flush to ensure any changes are
     *  propagated to the backing store.
     *  Commit fails if there is no transaction in progress.
     */
    void commit();

    /** Rolls back pending transaction.
     *  Rollback fails if there is no transaction in progress.
     */
    void rollback();

    /** Merge in tile sets.
     *
     * Only parts covered by in sets are affected.
     *
     * It is recommended to call this function inside a transaction.
     *
     * \param kept tile set that were already merged in this tile set
     * \param out tile sets to merge in
     */
    void mergeIn(const list &kept, const list &update);

    /** Merge out tile sets.
     *
     * Only parts covered by out sets are affected.
     *
     * It is recommended to call this function inside a transaction.
     *
     * \param kept tile set that were already merged in this tile set
     * \param out tile sets to merge out
     */
    void mergeOut(const list &kept, const list &update);

private:
    TileSet(const std::shared_ptr<Driver> &driver);

    struct Detail;
    std::unique_ptr<Detail> detail_;
    Detail& detail() { return *detail_; }
    const Detail& detail() const { return *detail_; }
public:
    /** Needed to instantiate.
     */
    class Factory; friend class Factory;

    struct Accessor; friend class Accessor;
};


// inline stuff

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

#endif // vadstena_libs_tilestorage_tileset_hpp_included_
