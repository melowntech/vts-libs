/**
 * \file vts/tileset.hpp
 * \author Vaclav Blazek <vaclav.blazek@citationtech.net>
 *
 * Tile set access.
 */

#ifndef vadstena_libs_vts_tileset_detail_hpp_included_
#define vadstena_libs_vts_tileset_detail_hpp_included_

#include "../tileset.hpp"
#include "../tileindex.hpp"
#include "./driver.hpp"

namespace vadstena { namespace vts {

typedef std::map<std::string, math::Extents2> SpatialDivisionExtents;

bool check(const SpatialDivisionExtents &l, const SpatialDivisionExtents &r);

struct TileSet::Properties : TileSetProperties {
    /** Data version/revision. Should be increment anytime the data change.
     *  Used in template URL's to push through caches.
     */
    unsigned int revision;

    // driver options
    driver::Options driverOptions;

    /** Range of lods where are tiles with mesh and/or atlas.
     */
    storage::LodRange lodRange;

    /** Extents (inclusive) of tiles with mesh and/or atlas at lodRange.min
     */
    TileRange tileRange;

    /** Occupied extents for each spatial division SRS
     */
    SpatialDivisionExtents spatialDivisionExtents;

    Properties() : revision(0) {}

    Properties(const TileSetProperties &slice)
        : TileSetProperties(slice), revision(0)
    {}
};

struct TileNode {
    MetaTile *metatile;
    const MetaNode *metanode;

    typedef std::map<TileId, TileNode> map;

    TileNode() : metanode() {}
    TileNode(MetaTile *metatile, const MetaNode *metanode)
        : metatile(metatile), metanode(metanode)
    {}

    void update(const TileId &tileId, const MetaNode &mn)
    {
        metatile->update(tileId, mn);
    }

    void set(const TileId &tileId, const MetaNode &mn)
    {
        metatile->set(tileId, mn);
    }
};

typedef std::map<TileId, MetaTile> MetaTiles;

/** Driver that implements physical aspects of tile set.
 */
struct TileSet::Detail
{
    bool readOnly;

    Driver::pointer driver;

    Properties properties;       // current properties
    mutable bool propertiesChanged; // marks that properties have been changed
    mutable bool metadataChanged;   // marks that metadata have been changed
    bool changed() const { return metadataChanged || propertiesChanged; }

    registry::ReferenceFrame referenceFrame;

    mutable TileNode::map tileNodes;
    mutable MetaTiles metaTiles;

    /** Index of existing tiles.
     */
    mutable TileIndex tileIndex;

    /** References to other datasets.
     */
    TileIndex references;

    LodRange lodRange;

    Detail(const Driver::pointer &driver);
    Detail(const Driver::pointer &driver
           , const TileSet::Properties &properties);
    ~Detail();

    void loadConfig();
    static Properties loadConfig(const Driver &driver);

    void saveConfig();

    void watch(utility::Runnable *runnable);

    void checkValidity() const;

    MetaTile* findMetaTile(const TileId &tileId, bool addNew = false) const;
    TileNode* findNode(const TileId &tileId, bool addNew = false) const;
    const MetaNode* findMetaNode(const TileId &tileId) const;

    void loadTileIndex();
    void saveTileIndex();

    void setTile(const TileId &tileId, const Tile &tile
                 , const NodeInfo *nodeInfo = nullptr);

    void setTile(const TileId &tileId, const TileSource &tile
                 , const NodeInfo *nodeInfo = nullptr);

    void setNavTile(const TileId &tileId, const NavTile &navtile);

    void setReferenceTile(const TileId &tileId, uint8_t other
                          , const NodeInfo *nodeInfo = nullptr);

    std::uint8_t metaOrder() const;
    TileId metaId(TileId tileId) const;

    void save(const OStream::pointer &os, const Mesh &mesh) const;
    void save(const OStream::pointer &os, const Atlas &atlas) const;
    void save(const OStream::pointer &os, const NavTile &navtile) const;

    void load(const IStream::pointer &os, Mesh &mesh) const;
    void load(const IStream::pointer &os, Atlas &atlas) const;
    void load(const NavTile::HeightRange &heightRange
              , const IStream::pointer &os, NavTile &navtile) const;

    MetaTile* addNewMetaTile(const TileId &tileId) const;

    TileNode* updateNode(TileId tileId, const MetaNode &metanode
                         , bool watertight);

    bool exists(const TileId &tileId) const;
    Mesh getMesh(const TileId &tileId) const;
    void getAtlas(const TileId &tileId, Atlas &atlas) const;
    void getNavTile(const TileId &tileId, NavTile &navtile) const;

    /** Trusts that findMetaNode(tileId) yields the same value as node.
     */
    Mesh getMesh(const TileId &tileId, const MetaNode *node) const;
    void getAtlas(const TileId &tileId, Atlas &atlas
                  , const MetaNode *node) const;
    void getNavTile(const TileId &tileId, NavTile &navtile
                    , const MetaNode *node) const;

    TileSource getTileSource(const TileId &metaId) const;

    void flush();
    void saveMetadata();

    MapConfig mapConfig() const;

    static MapConfig mapConfig(const Driver &driver);

    static MapConfig mapConfig(const Properties &properties
                               , const ExtraTileSetProperties &extra);

    ExtraTileSetProperties loadExtraConfig() const;
    static ExtraTileSetProperties loadExtraConfig(const Driver &driver);

    Detail& other(TileSet &otherSet) {
        return otherSet.detail();
    }

    const Detail& other(const TileSet &otherSet) const {
        return otherSet.detail();
    }

    bool fullyCovered(const TileId &tileId) const;

    void setPosition(const registry::Position &position);
    void addCredits(const registry::IdSet &credits);
    void addBoundLayers(const registry::IdSet &boundLayers);

private:
    void updateProperties(const MetaNode &metanode);
    void updateProperties(const NodeInfo &nodeInfo);
    void updateProperties(const Mesh &mesh);
};

inline void TileSet::Detail::checkValidity() const {
    if (!driver) {
        LOGTHROW(err2, storage::NoSuchTileSet)
            << "Tile set <" << properties.id << "> was removed.";
    }
}

inline std::uint8_t TileSet::Detail::metaOrder() const
{
    return referenceFrame.metaBinaryOrder;
}

inline TileId TileSet::Detail::metaId(TileId tileId) const
{
    tileId.x &= ~((1 << referenceFrame.metaBinaryOrder) - 1);
    tileId.y &= ~((1 << referenceFrame.metaBinaryOrder) - 1);
    return tileId;
}

inline const MetaNode* TileSet::Detail::findMetaNode(const TileId &tileId)
    const
{
    if (auto *node = findNode(tileId)) { return node->metanode; }
    return nullptr;
}

} } // namespace vadstena::vts

#endif // vadstena_libs_vts_tileset_detail_hpp_included_
