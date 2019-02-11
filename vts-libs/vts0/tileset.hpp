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
/**
 * \file vts/tileset.hpp
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

#ifndef vtslibs_vts0_tileset_hpp_included_
#define vtslibs_vts0_tileset_hpp_included_

#include <memory>
#include <cmath>
#include <list>
#include <string>
#include <array>

#include <boost/optional.hpp>
#include <boost/noncopyable.hpp>
#include <boost/filesystem/path.hpp>

#include "utility/runnable.hpp"

#include "types.hpp"
#include "tileindex.hpp"
#include "properties.hpp"

namespace vtslibs { namespace vts0 {

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

    /** Returns metadata of existing tile.
     * \param tileId idetifier of tile read
     * \return tile's metadata
     * \throw Error if tile doesn't exist
     */
    MetaNode getMetadata(const TileId &tileId) const;

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
                             , SettableProperties::MaskType mask
                             = ~SettableProperties::MaskType(0));

    /** Flush all pending changes to backing store (e.g. filesystem).
     *
     * If tile set is not allowed to be changed (i.e. open in read-only mode)
     * flush call is ignored.
     */
    void flush();

    /** Starts new transaction. (R/W)
     */
    void begin(utility::Runnable *runnable = nullptr);

    /** Commits pending transaction. Calls flush to ensure any changes are
     *  propagated to the backing store.
     *  Commit fails if there is no transaction in progress.
     */
    void commit();

    /** Rolls back pending transaction.
     *  Rollback fails if there is no transaction in progress.
     */
    void rollback();

    /** Starts watching runnable without entering a transaction.
     */
    void watch(utility::Runnable *runnable);

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

    /** Returns lod range covered by tiles.
     */
    LodRange lodRange() const;

    /** Tileset statistics.
     */
    struct Statistics {
        std::size_t tileCount;
        std::size_t metatileCount;
    };

    /** Returns tileset statistics.
     *
     *  NB: tileset must be flushed, otherwise a TileSetNotFlushed exception is
     *  thrown.
     */
    Statistics stat() const;

    // extended API; not to be used by general public :)
    class AdvancedApi; friend class AdvancedApi;

    AdvancedApi advancedApi();

    /** Internals. Public to ease library developers' life, not to allow users
     *  to put their dirty hands in the tileset's guts!
     */
    struct Detail;

private:
    TileSet(const std::shared_ptr<Driver> &driver);

    TileSet(const std::shared_ptr<Driver> &driver
            , const CreateProperties &properties);

    std::unique_ptr<Detail> detail_;
    Detail& detail() { return *detail_; }
    const Detail& detail() const { return *detail_; }

public:
    /** Needed to instantiate.
     */
    class Factory; friend class Factory;

    struct Accessor; friend class Accessor;
};

} } // namespace vtslibs::vts0

#endif // vtslibs_vts0_tileset_hpp_included_
