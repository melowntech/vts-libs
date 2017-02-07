/**
 * \file vts/storage/detail.hpp
 * \author Vaclav Blazek <vaclav.blazek@citationtech.net>
 *
 * Storage access (internals).
 */

#ifndef vadstena_libs_vts_storage_detail_hpp_included_
#define vadstena_libs_vts_storage_detail_hpp_included_

#include <boost/filesystem/path.hpp>

#include "../storage.hpp"
#include "../../storage/streams.hpp"

namespace vadstena { namespace vts {

using vadstena::storage::FileStat;

/** Tileset/glue Trash bin.
 */
class TrashBin {
public:
    struct Item {
        unsigned int revision;

        typedef std::map<TilesetIdList, Item> map;
    };

    TrashBin() {}

    void add(const TilesetIdList &id, const Item &item);
    void add(const TilesetId &id, const Item &item) {
        add(TilesetIdList(1, id), item);
    }

    void remove(const TilesetIdList &id);
    void remove(const TilesetId &id) { return remove(TilesetIdList(1, id)); }

    const Item* find(const TilesetIdList &id) const;
    const Item* find(const TilesetId &id) const {
        return find(TilesetIdList(1, id));
    }

    Item::map::const_iterator begin() const { return content_.begin(); }
    Item::map::const_iterator end() const { return content_.end(); }

private:
    Item::map content_;
};

struct Storage::Properties : StorageProperties {
    /** Data version/revision. Should be incremented anytime the data change.
     *  Used in template URL's to push through caches.
     */
    unsigned int revision;

    /** List of tilesets in this storage.
     */
    StoredTileset::list tilesets;

    /** List of glues
     */
    Glue::map glues;

    /** List of pending glues.
     */
    Glue::IdSet pendingGlues;

    /** List of empty glues.
     */
    Glue::IdSet emptyGlues;

    /** List of virtual surfaces
     */
    VirtualSurface::map virtualSurfaces;

    /** Information about removed tilesets/glues.
     */
    TrashBin trashBin;

    Properties() : revision(0) {}

    StoredTileset* findTileset(const TilesetId &tilesetId);
    const StoredTileset* findTileset(const TilesetId &tilesetId) const;

    StoredTileset::list::iterator findTilesetIt(const TilesetId &tilesetId);
    StoredTileset::list::const_iterator
    findTilesetIt(const TilesetId &tilesetId) const;

    bool hasTileset(const TilesetId &tileset) const {
        return findTileset(tileset);
    }

    int lastVersion(const TilesetId &baseId) const;

    Glue::map::iterator findGlue(const Glue::Id& glue);
    Glue::map::const_iterator findGlue(const Glue::Id& glue) const;

    bool hasGlue(const Glue::Id& glue) const {
        return findGlue(glue) != glues.end();
    }

    const Glue* getGlue(const Glue::Id& glue) const {
        auto fglues(findGlue(glue));
        return (fglues != glues.end()) ? &fglues->second : nullptr;
    }

    bool hasPendingGlue(const Glue::Id &glue) const {
        return pendingGlues.find(glue) != pendingGlues.end();
    }

    bool hasEmptyGlue(const Glue::Id &glue) const {
        return emptyGlues.find(glue) != emptyGlues.end();
    }

    VirtualSurface::map::iterator
    findVirtualSurface(const VirtualSurface::Id& virtualSurface);
    VirtualSurface::map::const_iterator
    findVirtualSurface(const VirtualSurface::Id& virtualSurface) const;

    bool hasVirtualSurface(const VirtualSurface::Id& virtualSurface) const {
        return findVirtualSurface(virtualSurface) != virtualSurfaces.end();
    }

    const VirtualSurface*
    getVirtualSurface(const VirtualSurface::Id& virtualSurface) const {
        auto fvirtualSurfaces(findVirtualSurface(virtualSurface));
        return (fvirtualSurfaces != virtualSurfaces.end())
            ? &fvirtualSurfaces->second : nullptr;
    }

    /** Return set of unique tilesets.
     *
     *  Either use tileset specified in subset or choose tilesets with highest
     *  version. If subset contains tilesets with same base use highest version
     *  as well.
     */
    TilesetIdSet unique(const TilesetIdSet *subset) const;

    /** For each tileset compute number of pending glues where tileset is at the
     *  top of the stack
     */
    TilesetIdCounts pendingGlueCount(TilesetIdSet tilesets) const;

    /** Normalize glue ID.
     */
    Glue::Id normalize(const Glue::Id &id) const;
};

TilesetIdList tilesetIdList(const StoredTileset::list &tilesets);

typedef std::vector<int> GlueIndices;
GlueIndices buildGlueIndices(const TilesetIdList &world, const Glue::Id &id);

struct Storage::Detail
{
    bool readOnly;

    boost::filesystem::path root;
    boost::filesystem::path configPath;
    boost::filesystem::path extraConfigPath;

    Properties properties;

    registry::ReferenceFrame referenceFrame;

    /** Information about root when tileset was open in read-only mode.
     */
    FileStat rootStat;

    /** Information about config when tileset was open in read-only mode.
     */
    FileStat configStat;

    /** Information about extra-config when tileset was open in read-only mode.
     */
    FileStat extraConfigStat;

    /** Time of last modification (recorded at read-only open)
     */
    std::time_t lastModified;

    Detail(const boost::filesystem::path &root
           , const StorageProperties &properties
           , CreateMode mode);

    Detail(const boost::filesystem::path &root
           , OpenMode mode);

    ~Detail();

    void loadConfig();

    static Storage::Properties loadConfig(const boost::filesystem::path &root);

    ExtraStorageProperties loadExtraConfig() const;

    static ExtraStorageProperties
    loadExtraConfig(const boost::filesystem::path &root);

    void saveConfig();

    /** Set and save new properties.
     */
    void saveConfig(const Properties &nProperties);

    TileSet open(const TilesetId &tilesetId) const;

    TileSet open(const Glue &glue) const;

    void add(const TileSet &tileset, const Location &where
             , const TilesetId &tilesetId, const AddOptions &addOptions);

    void readd(const TilesetId &tilesetId, const AddOptions &addOptions);

    void generateGlues(const TilesetId &tilesetId
                       , const AddOptions &addOptions);

    void generateGlue(const Glue::Id &glueId
                      , const AddOptions &addOptions);

    void remove(const TilesetIdList &tilesetIds);

    void createVirtualSurface(const TilesetIdSet &tilesets, CreateMode mode);

    void removeVirtualSurface(const TilesetIdSet &tilesets);

    std::tuple<Properties, StoredTileset>
    addTileset(const Properties &properties, const TilesetId &tilesetId
               , const AddOptions &addOptions
               , const Location &where) const;

    /** Removes given tileset from properties and returns new properties and
     *  list of removed glues and virtual surfaces.
     */
    std::tuple<Properties, Glue::map, VirtualSurface::map>
    removeTilesets(const Properties &properties
                   , const TilesetIdList &tilesetIds)
        const;

    /** Removes given virtual surfaces from properties and returns new
     *  properties and list of removed virtual surfaces.
     */
    std::tuple<Properties, VirtualSurface::map>
    removeVirtualSurfaces(const Properties &properties
                          , const VirtualSurface::Ids &ids)
        const;

    bool externallyChanged() const;

    void updateTags(const TilesetId &tilesetId, const Tags &add
                    , const Tags &remove);

    MapConfig mapConfig() const;

    static MapConfig mapConfig(const boost::filesystem::path &path);

    static MapConfig mapConfig(const boost::filesystem::path &root
                               , const Properties &properties
                               , const ExtraStorageProperties &extra
                               , const TilesetIdSet *subset = nullptr
                               , const TilesetIdSet *freeLayers = nullptr
                               , const boost::filesystem::path &prefix
                               = boost::filesystem::path());

    Glue::IdSet pendingGlues(const TilesetIdSet *subset) const;
};

} } // namespace vadstena::vts

#endif // vadstena_libs_vts_storage_detail_hpp_included_
