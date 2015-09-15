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

struct TileSet::Properties : StaticProperties {
    // driver options
    driver::Options driverOptions;

    Properties() {}
};

struct TileNode {
    MetaTile *metatile;
    const MetaNode *metanode;
    bool watertight;

    typedef std::map<TileId, TileNode> map;

    TileNode() : metanode(), watertight() {}
    TileNode(MetaTile *metatile, const MetaNode *metanode
             , bool watertight)
        : metatile(metatile), metanode(metanode), watertight(watertight)
    {}

    void update(const TileId &tileId, const MetaNode &mn
                , bool watertight)
    {
        metatile->update(tileId, mn);
        this->watertight = watertight;
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

    Properties savedProperties;  // properties as are on disk
    Properties properties;       // current properties
    bool propertiesChanged;      // marks whether properties have been changed

    storage::ReferenceFrame referenceFrame;

    mutable TileNode::map tileNodes;
    mutable MetaTiles metaTiles;

    bool metadataChanged;

    TileIndex tileIndex;
    TileIndex watertightIndex;
    TileIndex metaIndex;

    LodRange lodRange;

    Detail(const Driver::pointer &driver);
    Detail(const Driver::pointer &driver, const StaticProperties &properties);
    ~Detail();

    void loadConfig();

    void saveConfig();

    void watch(utility::Runnable *runnable);

    void checkValidity() const;

    MetaTile* findMetaTile(const TileId &tileId, bool addNew = false) const;
    TileNode* findNode(const TileId &tileId, bool addNew = false) const;

    void loadTileIndex();
    void saveTileIndex(const TileIndex &nTileIndex
                       , const TileIndex &nWatertightIndex
                       , const TileIndex &nMetaIndex);

    void setTile(const TileId &tileId, const Mesh &mesh, bool watertight
                 , const Atlas *atlas, const NavTile *navtile);

    std::uint8_t metaOrder() const;
    TileId metaId(TileId tileId) const;
    TileId originFromMetaId(TileId tileId) const;

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
};

inline void TileSet::DetailDeleter::operator()(Detail *d) { delete d; }

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
    tileId.x >>= referenceFrame.metaBinaryOrder;
    tileId.y >>= referenceFrame.metaBinaryOrder;
    return tileId;
}

inline TileId TileSet::Detail::originFromMetaId(TileId tileId) const
{
    tileId.x <<= referenceFrame.metaBinaryOrder;
    tileId.y <<= referenceFrame.metaBinaryOrder;
    return tileId;
}

} } // namespace vadstena::vts

#endif // vadstena_libs_vts_tileset_detail_hpp_included_
