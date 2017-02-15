#include <new>
#include <queue>
#include <set>

#include "dbglog/dbglog.hpp"
#include "utility/progress.hpp"
#include "jsoncpp/json.hpp"

#include "../tilestorage.hpp"
#include "./tileindex.hpp"
#include "./io.hpp"
#include "./tileop.hpp"
#include "./driver/flat.hpp"

namespace vtslibs { namespace tilestorage {

typedef std::map<TileId, MetaNode> Metadata;

typedef std::set<TileId> TileIdSet;

struct TileSet::Detail {
    Driver::pointer driver;

    // properties
    Properties savedProperties;  // properties as are on disk
    Properties properties;       // current properties
    bool propertiesChanged;      // marks whether properties have been changed

    TileIndex tileIndex;    // tile index that reflects state on the disk
    TileIndex metaIndex;    // metatile index that reflects state on the disk

    Extents extents;              // extents covered by tiles
    LodRange lodRange;            // covered lod range
    mutable Metadata metadata;    // all metadata as in-memory structure
    mutable TileIdSet loadedMetatiles; // marks that given tiles are loaded
    bool metadataChanged;         // marks whether metadata have been changed

    bool tx; // pending transaction?

    Detail(const Driver::pointer &driver);
    Detail(const Driver::pointer &driver, const CreateProperties &properties);

    ~Detail();

    void checkValidity() const;

    void check(const TileId &tileId) const;

    void checkTx(const std::string &action) const;

    void loadConfig();

    void saveConfig();

    void saveMetadata();

    void loadTileIndex();

    Tile getTile(const TileId &tileId) const;

    boost::optional<Tile> getTile(const TileId &tileId, std::nothrow_t) const;

    MetaNode setTile(const TileId &tileId, const Mesh &mesh
                     , const Atlas &atlas, const TileMetadata *metadata
                     , const boost::optional<double> &pixelSize = boost::none);

    MetaNode removeTile(const TileId &tileId);

    MetaNode* loadMetatile(const TileId &tileId) const;

    void loadMetatileFromFile
        (Metadata &metadata, const TileId &tileId
         , const MetaNodeNotify &notify = MetaNodeNotify())
        const;

    MetaNode* findMetaNode(const TileId &tileId) const;

    MetaNode setMetaNode(const TileId &tileId, const MetaNode& metanode);

    void setMetadata(const TileId &tileId, const TileMetadata& metadata);

    MetaNode& createVirtualMetaNode(const TileId &tileId);

    bool isFoat(const TileId &tileId) const;

    void updateTreeMetadata(const TileId &tileId);

    void updateTreeMetadata(const TileId &tileId, MetaNode &metanode);

    void updateTree(const TileId &tileId);

    void updateTree(const TileId &tileId, MetaNode &metanode);

    void flush();

    void purgeMetadata();

    void removeOverFoat();

    void dropRemovedMetatiles(const TileIndex &before, const TileIndex &after);

    void saveMetatiles(TileIndex &tileIndex, TileIndex &metaIndex) const;

    void begin(utility::Runnable *runnable);

    void commit();

    void rollback();

    void watch(utility::Runnable *runnable);

    void fixDefaultPosition(const list &tileSets);

    TileId parent(const TileId &tileId) const;

    void clone(const Detail &src);

    void clone(const Detail &src, const CloneOptions::Filter &filter);

    void setFoat(const TileId &tileId);

    void resetFoat();

    /** Filters heightfield in area affected by bordering tiles of continuous
     *  area and bordering tiles of discrete tiles.
     *
     * Each entry in discrete list belongs to appropriate continuous list.
     *
     * \param continuous list of tile indices of continuous area
     * \param discrete list of tile indices of individual tiles
     * \param cutoff cut off period of CatmullRom2 (in both directions)
     */
    void filterHeightmap(const TileIndices &continuous
                         , const TileIndices *discrete = nullptr
                         , double cutoff = 2.0);

    Statistics stat() const;

    Detail& other(const TileSet::pointer &otherSet) {
        return otherSet->detail();
    }

    const Detail& other(const TileSet::pointer &otherSet) const {
        return otherSet->detail();
    }
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
        LOGTHROW(err2, storage::NoSuchTileSet)
            << "Tile set <" << properties.id << "> was removed.";
    }
}

} } // namespace vtslibs::tilestorage
