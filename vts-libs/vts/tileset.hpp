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

    Mesh getMesh(const TileId &tileId) const;
    void setMesh(const TileId &tileId, const Mesh &mesh
                 , bool waterproof = true);

    void getAtlas(const TileId &tileId, Atlas &atlas) const;
    void setAtlas(const TileId &tileId, const Atlas &atlas);

    void setTile(const TileId &tileId, const Tile &tile);

    MetaNode getMetaNode(const TileId &tileId) const;
    void setMetaNode(const TileId &tileId, const MetaNode node);

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
    storage::ReferenceFrame referenceFrame() const;

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
