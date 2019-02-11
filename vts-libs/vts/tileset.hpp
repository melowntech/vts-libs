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
 */

#ifndef vtslibs_vts_tileset_hpp_included_
#define vtslibs_vts_tileset_hpp_included_

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

#include "basetypes.hpp"
#include "types.hpp"
#include "mesh.hpp"
#include "metatile.hpp"
#include "atlas.hpp"
#include "tileset/properties.hpp"
#include "mapconfig.hpp"
#include "tileindex.hpp"
#include "tilesource.hpp"
#include "options.hpp"

namespace vtslibs { namespace vts {

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

    /** Adds given credits into list of credits used by this tileset.
     */
    void addCredits(const registry::IdSet &credits);

    /** Adds given bound layers into list of bound layers used by this tileset.
     */
    void addBoundLayers(const registry::IdSet &boundLayers);

    /** Generates map configuration for this single tile set.
     * \param includeExtra include extra configuration in the output.
     * \return map configuration
     */
    MapConfig mapConfig(bool includeExtra = true) const;

    /** Generates meshTiles configutation
     */
    MeshTilesConfig meshTilesConfig(bool includeExtra = true) const;

    /** Returns tile's mesh.
     */
    Mesh getMesh(const TileId &tileId) const;

    /** Returns stured tile's mesh mask unless generate is set to true. In that case,
     *  fresh mesh mask is generated from mesh data.
     */
    MeshMask getMeshMask(const TileId &tileId, bool generate = false) const;

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

    /** Marks tile as influenced tile.
     *  Fails if tile is already a valid tile.
     */
    void markInfluencedTile(const TileId &tileId);

    /** Sets tile's navtile. Tile must already have mesh.
     *  Only navtile is stored.
     */
    void setNavTile(const TileId &tileId, const NavTile &navtile);

    /** Returns tile's navtile.
     */
    void getNavTile(const TileId &tileId, NavTile &navtile) const;

    /** Set's tile's surrogate value (i.e. representative height).
     */
    void setSurrogateValue(const TileId &tileId, float value);

    /** Returns tile's metanode.
     */
    MetaNode getMetaNode(const TileId &tileId) const;

    /** Returns tile's metanode.
     */
    const MetaNode* getMetaNode(const TileId &tileId, const std::nothrow_t&)
        const;

    /** Returns metatile.
     */
    MetaTile getMetaTile(const TileId &metaId) const;

    /** Shows metatile version.
     */
    int getMetaTileVersion(const TileId &metaId) const;

    /** Returns ID of metatile tileId belongs to.
     */
    TileId metaId(const TileId &tileId) const;

    /** Returns tile's content source.
     */
    TileSource getTileSource(const TileId &tileId) const;

    /** Returns reference for given tile.
     *  Reference is number > 0. 0 means no reference.
     */
    int getReference(const TileId &tileId) const;

    /** Checks whether tile exist.
     */
    bool exists(const TileId &tileId) const;

    /** Get source reference. Returns zero if tile does not exist. For tileset
     *  without stored tileset reference always returns 1 for existing tile.
     */
    int sourceReference(const TileId &tileId) const;

    /** Checks whether tile extist and covers whole tile.
     */
    bool fullyCovered(const TileId &tileId) const;

    /** Flushes tileset.
     *
     * Must be called before close otherwise tileset is useless.
     */
    void flush();

    /** Empties any cached content at once. Fails if there are any modified data.
     */
    void emptyCache() const;

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

    /** Returns lod range at given lod. Calculated on the fly.
     */
    TileRange tileRange(Lod lod) const;

    /** Referce frame in charge.
     */
    const registry::ReferenceFrame& referenceFrame() const;

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

    /** Derives metatile index from tile index
     */
    TileIndex metaIndex() const;

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

    typedef std::vector<TileSet> list;

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

    /** Pastes other tileset into this one.
     */
    void paste(const TileSet &src
               , const boost::optional<LodRange> &lodRange = boost::none);

    /** Returns type information.
     */
    std::string typeInfo() const;

    /** Get node info for given tile.
     */
    NodeInfo nodeInfo(const TileId &tileId) const;

    /** Returns representative texel size. Computed on the fly from most
     *  detailed LOD's metanodes.
     */
    double texelSize() const;

    /** Returns root node.
     */
    NodeInfo rootNode() const;

    /** Returns mapConfig for given path.
     *
     * \param root root path of all datasets (surfaces and glues)
     * \param includeExtra include extra configuration in the output.
     * \return map configuration
     */
    static MapConfig mapConfig(const boost::filesystem::path &root
                               , bool includeExtra = true);

    /** Returns mapConfig for given driver.
     *
     * \param driver data source
     * \param includeExtra include extra configuration in the output.
     * \return map configuration
     */
    static MapConfig mapConfig(const Driver &driver, bool includeExtra);

    /** Returns meshTiles config for given path.
     *
     * \param root root path of all datasets (surfaces and glues)
     * \return map configuration
     */
    static MeshTilesConfig meshTilesConfig(const boost::filesystem::path &root
                                           , bool includeExtra = true);

    /** Returns meshTiles config for given driver.
     *
     * \param driver data source
     * \return map configuration
     */
    static MeshTilesConfig meshTilesConfig(const Driver &driver
                                           , bool includeExtra);

    /** Check for tileset at given path.
     */
    static bool check(const boost::filesystem::path &root);

    /** Check for tileset at given path.
     */
    static bool check(const boost::filesystem::path &root
                      , const std::string &mime);

    /** Low level access: open driver directly.
     *  You need to include tileset/driver.hpp to access it.
     */
    static std::shared_ptr<Driver>
    openDriver(const boost::filesystem::path &root
               , const OpenOptions &openOptions = OpenOptions());

    static void relocate(const boost::filesystem::path &root
                         , const RelocateOptions &options
                         , const std::string &prefix = "");

    /** Recursively reencode datasets:
     *  plain: reencoded
     *  local: recursive reencode, revision bump on success
     *  remote: revision bump
     *  local: recursive reencode, revision bump on success
     */
    static void reencode(const boost::filesystem::path &root
                         , const ReencodeOptions &options
                         , const std::string &prefix = "");

    /** Creates glue from given sets.
     *
     * \param glue output tileset for glue tiles
     * \param sets input tilesets
     * \param options glue creation options
     *
     *  Priority grows from left to right.
     *
     */
    static void createGlue(TileSet &glue, const list &sets
                           , const GlueCreationOptions &options);

    /** Glue statistics returned by analyzeGlue.
     */
    struct GlueStatistics {
        std::size_t tilesToGenerate;

        GlueStatistics() : tilesToGenerate() {}
    };

    /** Updates revision to be at least equal to the provided number.
     */
    unsigned int ensureRevision(unsigned int revision);

    /** Tries to get revision from other tileset. If such revision can be read
     *  then calls ensureRevision(oldRevision + 1).
     */
    unsigned int ensureRevision(const boost::filesystem::path &root);

    /** Analyze glue from given sets.
     *
     * \param sets input tilesets
     * \param options glue creation options
     *
     *  Priority grows from left to right.
     *
     */
    static GlueStatistics analyzeGlue(const list &sets
                                      , const GlueCreationOptions &options);

    /** Internals. Public to ease library developers' life, not to allow users
     *  to put their dirty hands in the tileset's guts!
     */
    struct Detail;

    /** Full tileset properties.
     */
    typedef FullTileSetProperties Properties;

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

    /** Driver helper.
     */
    friend class Driver;
};

/** Low-level create operation.
 */
TileSet createTileSet(const boost::filesystem::path &path
                      , const TileSet::Properties &properties
                      , CreateMode mode);

} } // namespace vtslibs::vts

#endif // vtslibs_vts_tileset_hpp_included_
