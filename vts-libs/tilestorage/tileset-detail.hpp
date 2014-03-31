#include <new>
#include <queue>
#include <set>

#include "dbglog/dbglog.hpp"
#include "jsoncpp/json.hpp"

#include "../tilestorage.hpp"
#include "./tileindex.hpp"
#include "./io.hpp"
#include "./tileop.hpp"
#include "./driver/flat.hpp"

namespace vadstena { namespace tilestorage {

typedef std::map<TileId, MetaNode> Metadata;

typedef std::set<TileId> TileIdSet;

struct TileSet::Detail {
    Driver::pointer driver;

    // properties
    Properties savedProperties;  // properties as are on disk
    Properties properties;       // current properties
    bool propertiesChanged; // marks whether properties have been changed

    TileIndex tileIndex;    // tile index that reflects state on the disk
    TileIndex metaIndex;    // metatile index that reflects state on the disk

    Extents extents;              // extents covered by tiles
    LodRange lodRange;            // covered lod range
    mutable Metadata metadata;    // all metadata as in-memory structure
    mutable TileIdSet loadedMetatiles; // marks that given tiles are loaded
    bool metadataChanged;         // marks whether metadata have been changed

    mutable Json::Value config;

    bool tx; // pending transaction?

    Detail(const Driver::pointer &driver);
    Detail(const Driver::pointer &driver, const CreateProperties &properties);

    ~Detail();

    void checkValidity() const;

    void check(const TileId &tileId) const;

    void loadConfig();

    void saveConfig();

    void saveMetadata();

    void loadTileIndex();

    Tile getTile(const TileId &tileId) const;

    boost::optional<Tile> getTile(const TileId &tileId, std::nothrow_t) const;

    MetaNode setTile(const TileId &tileId, const Mesh &mesh
                     , const Atlas &atlas, const TileMetadata *metadata);

    MetaNode removeTile(const TileId &tileId);

    MetaNode* loadMetatile(const TileId &tileId) const;

    void loadMetatileFromFile(const TileId &tileId) const;

    MetaNode* findMetaNode(const TileId &tileId) const;

    MetaNode setMetaNode(const TileId &tileId, const MetaNode& metanode);

    void setMetadata(const TileId &tileId, const TileMetadata& metadata);

    MetaNode& createVirtualMetaNode(const TileId &tileId);

    bool isFoat(const TileId &tileId) const;

    void updateTree(const TileId &tileId);

    void updateTree(const TileId &tileId, MetaNode &metanode);

    void flush();

    void purgeMetadata();

    void saveMetatiles(TileIndex &tileIndex, TileIndex &metaIndex) const;

    void begin();

    void commit();

    void rollback();

    /** Merge subtree starting at index.
     *  Calls itself recursively.
     *
     * NB If remove tile set must have same dimensions as generate tile set (it
     * non-null).
     */
    void mergeSubtree(const TileIndex &generate, const TileIndex *remove
                      , const Index &index, const TileSet::list &src
                      , const Tile &parentTile = Tile(), int quadrant = -1
                      , bool parentGenerated = false);

    /** Generates new tile as a merge of tiles from other tilesets.
     */
    Tile generateTile(const TileId &tileId, const TileSet::list &src
                      , const Tile &parentTile, int quadrant);

    void fixDefaultPosition(const list &tileSets);

    TileId parent(const TileId &tileId) const;

    void clone(const Detail &src);
};


// inline stuff

inline TileId TileSet::Detail::parent(const TileId &tileId) const
{
    return tilestorage::parent(properties.alignment
                               , properties.baseTileSize
                               , tileId);
}

inline void TileSet::Detail::checkValidity() const
{
    if (!driver) {
        LOGTHROW(err2, NoSuchTileSet)
            << "Tile set <" << properties.id << "> was removed.";
    }
}

} } // namespace vadstena::tilestorage
