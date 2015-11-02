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

struct Storage::Properties : StorageProperties {
    /** Data version/revision. Should be incremented anytime the data change.
     *  Used in template URL's to push through caches.
     */
    unsigned int revision;

    /** List of tilesets in this storage.
     */
    TilesetIdList tilesets;

    /** List of glues
     */
    Glue::map glues;

    Properties() : revision(0) {}

    TilesetIdList::iterator findTileset(const std::string& tileset);
    TilesetIdList::const_iterator
    findTileset(const std::string& tileset) const;

    bool hasTileset(const std::string& tileset) const {
        return findTileset(tileset) != tilesets.end();
    }

    Glue::map::iterator findGlue(const Glue::Id& glue);
    Glue::map::const_iterator findGlue(const Glue::Id& glue) const;

    bool hasGlue(const Glue::Id& glue) const {
        return findGlue(glue) != glues.end();
    }
};

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

    void saveConfig();

    void add(const TileSet &tileset, const Location &where
             , const std::string tilesetId);

    void remove(const TilesetIdList &tilesetIds);

    Properties addTileset(const Properties &properties
                          , const std::string tilesetId
                          , const Location &where) const;

    /** Removes given tileset from properties and returns new properties and
     *  list of removed glues.
     */
    std::tuple<Properties, Glue::map>
    removeTilesets(const Properties &properties
                   , const TilesetIdList &tilesetIds)
        const;

    bool externallyChanged() const;

    MapConfig mapConfig() const;

    static MapConfig mapConfig(const boost::filesystem::path &path);

    static MapConfig mapConfig(const boost::filesystem::path &root
                               , const Storage::Properties &properties);
};

inline void Storage::DetailDeleter::operator()(Detail *d) { delete d; }

} } // namespace vadstena::vts

#endif // vadstena_libs_vts_storage_detail_hpp_included_
