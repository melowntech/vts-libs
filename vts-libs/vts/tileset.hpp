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
#include "../storage/resources.hpp"

#include "./basetypes.hpp"
#include "./types.hpp"
#include "./mesh.hpp"
#include "./metatile.hpp"
#include "./atlas.hpp"
#include "./tileset/properties.hpp"
#include "./mapconfig.hpp"
#include "./tileindex.hpp"
#include "./tilesource.hpp"

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
    TileSetProperties getProperties() const;

    /** Sets new position.
     */
    void setPosition(const registry::Position &position);

    /** Generates map configuration for this single tile set.
     */
    MapConfig mapConfig() const;

    /** Returns tile's mesh.
     */
    Mesh getMesh(const TileId &tileId) const;

    /** Returns tiles' atlas.
     */
    void getAtlas(const TileId &tileId, Atlas &atlas) const;

    /** Set tile content.
     *  \param tileId tile identifier
     *  \param tile tile content
     */
    void setTile(const TileId &tileId, const Tile &tile);

    /** Set tile content.
     *
     * Supplied nodeInfo must be correct!
     *
     *  \param tileId tile identifier
     *  \param tile tile content
     *  \param nodeInfo information about node
     */
    void setTile(const TileId &tileId, const Tile &tile
                 , const NodeInfo &nodeInfo);

    /** Set tile content.
     *  \param tileId tile identifier
     *  \param tile tile content source
     */
    void setTile(const TileId &tileId, const TileSource &tile);

    /** Set tile content.
     *
     * Supplied nodeInfo must be correct!
     *
     *  \param tileId tile identifier
     *  \param tile tile content source
     *  \param nodeInfo information about node
     */
    void setTile(const TileId &tileId, const TileSource &tile
                 , const NodeInfo &nodeInfo);

    /** Returns tile's navtile.
     */
    void getNavTile(const TileId &tileId, NavTile &navtile) const;

    /** Returns tile's metanode.
     */
    MetaNode getMetaNode(const TileId &tileId) const;

    /** Returns metatile.
     */
    MetaTile getMetaTile(const TileId &metaId) const;

    /** Returns tile's content source.
     */
    TileSource getTileSource(const TileId &tileId) const;

    /** Checks whether tile extist.
     */
    bool exists(const TileId &tileId) const;

    /** Flushes tileset.
     *
     * Must be called before close otherwise tileset is useless.
     */
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

    /** Root path of this tileset.
     */
    boost::filesystem::path root() const;

    /** Get tileset's ID.
     */
    std::string id() const { return getProperties().id; }

    /** Returns tile index.
     */
    const TileIndex& tileIndex() const;

    /** Returns tile index in given lod range
     */
    TileIndex tileIndex(const LodRange &lodRange) const;

    /** Returns sphere of influence of this tileset.
     *
     *  SoI are all tiles that have giventype and also tiles above and below
     *  them.
     *
     * \param range optional LOD range in which to generate the SoI;
     *              defaults to tileset's LOD range
     * \param type type of tile to check
     * \return SoI where only influenced tile has non-zero value
     */
    TileIndex sphereOfInfluence(const LodRange &range = LodRange::emptyRange()
                                , TileIndex::Flag::value_type type
                                = TileIndex::Flag::mesh)
        const;

    typedef std::vector<const TileSet*> const_ptrlist;

    /** Creates glue from given sets into this set.
     *
     *  Priority grows from left to right.
     */
    void createGlue(const const_ptrlist &sets);

    bool externallyChanged() const;

    /** Returns time of last modification. Recorded at read-only open.
     */
    std::time_t lastModified() const;

    /** Returns information about used resources.
     */
    storage::Resources resources() const;

    /** Tells caller whether this tileset can contain given tile set.
     */
    bool canContain(const NodeInfo &nodeInfo) const;

    /** Returns mapConfig for given path.
     */
    static MapConfig mapConfig(const boost::filesystem::path &root);

    /** Check for tileset at given path.
     */
    static bool check(const boost::filesystem::path &root);

    /** Internals. Public to ease library developers' life, not to allow users
     *  to put their dirty hands in the tileset's guts!
     */
    struct Detail;

    /** Full tileset properties.
     */
    struct Properties;

private:
    TileSet(const std::shared_ptr<Driver> &driver
            , const TileSet::Properties &properties);
    TileSet(const std::shared_ptr<Driver> &driver);

    std::shared_ptr<Detail> detail_;

public:
    Detail& detail() { return *detail_; }
    const Detail& detail() const { return *detail_; }

    /** Needed to instantiate.
     */
    class Factory; friend class Factory;
};

} } // namespace vadstena::vts

#endif // vadstena_libs_vts_tileset_hpp_included_
