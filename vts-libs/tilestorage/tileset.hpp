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

#include <boost/optional.hpp>
#include <boost/noncopyable.hpp>
#include <boost/filesystem/path.hpp>

#include "./types.hpp"
#include "./tileindex.hpp"
#include "./properties.hpp"
#include "../ids.hpp"
#include "../range.hpp"

namespace vadstena { namespace tilestorage {

/** Driver that implements physical aspects of tile set.
 */
class Driver;

/** TileSet interface.
 */
class TileSet
    : private boost::noncopyable
    , public std::enable_shared_from_this<TileSet>
{
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
     * \param pixelSize metadata pixel size of tile
     *                  (optional, calculated from mesh and atlas otherwise)
     */
    void setTile(const TileId &tileId, const Mesh &mesh, const Atlas &atlas
                 , const TileMetadata *metadata = nullptr
                 , const boost::optional<double> &pixelSize = boost::none);

    /** Set new tile's metadata.
     * \param tileId idetifier of tile write
     * \param metadata new tile metadata
     * \throw Error if tile doesn't exist
     */
    void setMetadata(const TileId &tileId, const TileMetadata &metadata);

    /** Remove tile. Tile can be non-existent.
     * \param tileId idetifier of tile write
     */
    void removeTile(const TileId &tileId);

    /** Query for tile's existence.
     * \param tileId identifier of queried tile
     * \return true if given tile exists, false otherwise
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
                             , SettableProperties::MaskType mask = ~0ul);

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

    /** Check for pending transaction.
     */
    bool inTx() const;

    /** Is the tile set empty (i.e. has it no tile?)
     */
    bool empty() const;

    /** Remove storage.
     */
    void drop();

    /** Checks whether tile sets are compatible with this set (i.e. can be
     * merged into this set)
     *
     * \param sets tile sets to check
     * \return true if all sets can be merged into this set
     */
    bool compatible(const TileSet &other);

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

    /** Updates tileset. Currently, only embedded web browser is updated to
     *  compiled-in version.
     */
    void update();

    /** Pastes conent of supplied tilesets into this tileset.
     */
    void paste(const list &update);

    // extended API; not to be used by general public :)
    class AdvancedApi; friend class AdvancedApi;

    AdvancedApi advancedApi();

private:
    TileSet(const std::shared_ptr<Driver> &driver);

    TileSet(const std::shared_ptr<Driver> &driver
            , const CreateProperties &properties);

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

} } // namespace vadstena::tilestorage

#endif // vadstena_libs_tilestorage_tileset_hpp_included_
