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

    /** Range of lods where are tiles with mesh and/or atlas.
     */
    storage::LodRange lodRange;

    /** Extents (inclusive) of tiles with mesh and/or atlas at lodRange.min
     */
    TileRange tileRange;

    Properties() {}
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
    bool changed;                // marks whether tileset has been changed

    registry::ReferenceFrame referenceFrame;

    mutable TileNode::map tileNodes;
    mutable MetaTiles metaTiles;

    struct TileFlag {
        enum : std::uint8_t {
            mesh = 0x01
            , watertight = 0x02
            , atlas = 0x04
            , navtile = 0x08
            , meta = 0x10
        };
    };

    TileIndex tileIndex;

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
    void saveTileIndex();

    void setTile(const TileId &tileId, const Mesh *mesh
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

    void flush();
    void saveMetadata();

    MapConfig mapConfig() const;
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
    tileId.x &= ~((1 << referenceFrame.metaBinaryOrder) - 1);
    tileId.y &= ~((1 << referenceFrame.metaBinaryOrder) - 1);
    return tileId;
}

} } // namespace vadstena::vts

#endif // vadstena_libs_vts_tileset_detail_hpp_included_
