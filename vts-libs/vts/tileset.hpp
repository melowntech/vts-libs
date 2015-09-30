/**
 * \file vts/tileset.hpp
 * \author Vaclav Blazek <vaclav.blazek@citationtech.net>
 *
 * Tile set access.
 */

#ifndef vadstena_libs_vts_tileset_hpp_included_
#define vadstena_libs_vts_tileset_hpp_included_

#include <memory>
#include <cmath>
#include <list>
#include <string>
#include <array>

#include <boost/optional.hpp>
#include <boost/noncopyable.hpp>
#include <boost/filesystem/path.hpp>

#include "utility/runnable.hpp"

#include "../storage/lod.hpp"
#include "../storage/range.hpp"

#include "./basetypes.hpp"
#include "./types.hpp"
#include "./mesh.hpp"
#include "./metatile.hpp"
#include "./atlas.hpp"
#include "./tileset/properties.hpp"
#include "./mapconfig.hpp"

namespace vadstena { namespace vts {

/** Driver that implements physical aspects of tile set.
 */
class Driver;

/** TileSet interface.
 */
class TileSet {
public:
    ~TileSet();

    /** Get tile set propeties.
     * \return tile set properties
     */
    StaticProperties getProperties() const;

    /** Generates map configuration for this signle tile set.
     */
    MapConfig mapConfig() const;

    Mesh getMesh(const TileId &tileId) const;

    void getAtlas(const TileId &tileId, Atlas &atlas) const;

    void setTile(const TileId &tileId, const Tile &tile);

    void setNavTile(const TileId &tileId, const NavTile &navtile);

    void getNavTile(const TileId &tileId, NavTile &navtile) const;

    MetaNode getMetaNode(const TileId &tileId) const;

    MetaTile getMetaTile(const TileId &metaId) const;

    bool exists(const TileId &tileId) const;

    void flush();

    /** Starts watching runnable without entering a transaction.
     */
    void watch(utility::Runnable *runnable);

    /** Is the tile set empty (i.e. has it no tile?)
     */
    bool empty() const;

    /** Remove storage.
     */
    void drop();

    /** Returns lod range covered by tiles.
     */
    LodRange lodRange() const;

    /** Referce frame in charge.
     */
    registry::ReferenceFrame referenceFrame() const;

    /** Returns constant driver. Used in delivery system.
     */
    const Driver& driver() const;

    /** Internals. Public to ease library developers' life, not to allow users
     *  to put their dirty hands in the tileset's guts!
     */
    struct Detail;

private:
    TileSet(const std::shared_ptr<Driver> &driver
            , const StaticProperties &properties);
    TileSet(const std::shared_ptr<Driver> &driver);

    struct DetailDeleter { void operator()(Detail*); };
    std::unique_ptr<Detail, DetailDeleter> detail_;
    Detail& detail() { return *detail_; }
    const Detail& detail() const { return *detail_; }

public:
    /** Needed to instantiate.
     */
    class Factory; friend class Factory;

    struct Accessor; friend class Accessor;

    struct Properties;
};

} } // namespace vadstena::vts

#endif // vadstena_libs_vts_tileset_hpp_included_
