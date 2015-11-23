/**
 * \file vts/storage/storage.cpp
 * \author Vaclav Blazek <vaclav.blazek@citationtech.net>
 *
 * Tile set storage access.
 */

#include <memory>
#include <string>
#include <exception>
#include <algorithm>
#include <iterator>

#include <boost/optional.hpp>
#include <boost/noncopyable.hpp>
#include <boost/filesystem.hpp>
#include <boost/format.hpp>
#include <boost/range/adaptor/reversed.hpp>

#include "utility/streams.hpp"
#include "utility/guarded-call.hpp"
#include "utility/streams.hpp"

#include "../../storage/error.hpp"
#include "../storage.hpp"
#include "../../vts.hpp"
#include "./detail.hpp"
#include "../tileset/detail.hpp"

#include "./config.hpp"
#include "./paths.hpp"

namespace fs = boost::filesystem;

namespace vadstena { namespace vts {

namespace {
    const fs::path ConfigFilename("storage.conf");
    const fs::path ExtraConfigFilename("extra.conf");
}


void TrashBin::add(const TilesetIdList &id, const Item &item)
{
    content_[id] = item;
}

void TrashBin::remove(const TilesetIdList &id)
{
    content_.erase(id);
}

const TrashBin::Item* TrashBin::find(const TilesetIdList &id) const
{
    auto fcontent(content_.find(id));
    if (fcontent == content_.end()) { return nullptr; }
    return &fcontent->second;
}

Storage createStorage(const boost::filesystem::path &path
                      , const StorageProperties &properties
                      , CreateMode mode)
{
    return { path, properties, mode };
}

Storage openStorage(const boost::filesystem::path &path
                    , OpenMode mode)
{
    return { path, mode };
}

Storage::Storage(const boost::filesystem::path &path, OpenMode mode)
    : detail_(new Detail(path, mode))
{
}

Storage::Storage(const boost::filesystem::path &path
                 , const StorageProperties &properties
                 , CreateMode mode)
    : detail_(new Detail(path, properties, mode))
{}

Storage::~Storage()
{
    // no-op
}

Storage::Detail::~Detail()
{
}

Storage::Detail::Detail(const boost::filesystem::path &root
                        , const StorageProperties &properties
                        , CreateMode mode)
    : root(root)
    , configPath(root / ConfigFilename)
    , extraConfigPath(root / ExtraConfigFilename)
    , referenceFrame(registry::Registry::referenceFrame
                     (properties.referenceFrame))
{
    // fill in slice
    static_cast<StorageProperties&>(this->properties) = properties;

    if (!create_directories(root)) {
        // directory already exists -> fail if mode says so
        if (mode == CreateMode::failIfExists) {
            LOGTHROW(err2, vadstena::storage::StorageAlreadyExists)
                << "Storage " << root << " already exists.";
        }

        // OK, we can overwrite; cache contents of old config (if any)
        try {
            auto old(storage::loadConfig(configPath));
            this->properties.revision = old.revision + 1;
        } catch (...) {}
    }

    saveConfig();
}

Storage::Detail::Detail(const boost::filesystem::path &root
                        , OpenMode mode)
    : root(root)
    , configPath(root / ConfigFilename)
    , extraConfigPath(root / ExtraConfigFilename)
    , properties(storage::loadConfig(configPath))
    , rootStat(FileStat::stat(root))
    , configStat(FileStat::stat(configPath))
    , extraConfigStat(FileStat::stat(extraConfigPath, std::nothrow))
    , lastModified(std::max({ rootStat.lastModified, configStat.lastModified
                    , extraConfigStat.lastModified }))
{
    (void) mode;

    referenceFrame = registry::Registry::referenceFrame
        (properties.referenceFrame);
}

void Storage::Detail::loadConfig()
{
    properties = loadConfig(root);
}

Storage::Properties Storage::Detail::loadConfig(const fs::path &root)
{
    try {
        // load config
        const auto p(storage::loadConfig(root / ConfigFilename));
        return p;
    } catch (const std::exception &e) {
        LOGTHROW(err2, vadstena::storage::Error)
            << "Unable to read config: <" << e.what() << ">.";
    }
    throw;
}

void Storage::Detail::saveConfig()
{
    // save json
    try {
        storage::saveConfig(configPath, properties);
    } catch (const std::exception &e) {
        LOGTHROW(err2, vadstena::storage::Error)
            << "Unable to write config: <" << e.what() << ">.";
    }
}

bool Glue::references(const std::string &tilesetId) const
{
    return (std::find(id.begin(), id.end(), tilesetId) != id.end());
}

TilesetIdList::iterator
Storage::Properties::findTileset(const std::string& tileset)
{
    return std::find(tilesets.begin(), tilesets.end(), tileset);
}

TilesetIdList::const_iterator
Storage::Properties::findTileset(const std::string& tileset) const
{
    return std::find(tilesets.begin(), tilesets.end(), tileset);
}

Glue::map::iterator Storage::Properties::findGlue(const Glue::Id &glue)
{
    return glues.find(glue);
}

Glue::map::const_iterator
Storage::Properties::findGlue(const Glue::Id& glue) const
{
    return glues.find(glue);
}

TilesetIdList Storage::tilesets() const
{
    return detail().properties.tilesets;
}

Glue::map Storage::glues() const
{
    return detail().properties.glues;
}

MapConfig Storage::mapConfig() const
{
    return detail().mapConfig();
}

MapConfig Storage::mapConfig(const boost::filesystem::path &path)

{
    return Detail::mapConfig(path);
}

MapConfig Storage::Detail::mapConfig(const boost::filesystem::path &path)
{
    return mapConfig(path, loadConfig(path));
}

MapConfig Storage::Detail::mapConfig() const
{
    return mapConfig(root, properties);
}

MapConfig Storage::Detail::mapConfig(const boost::filesystem::path &root
                                     , const Storage::Properties &properties)
{
    auto referenceFrame(registry::Registry::referenceFrame
                        (properties.referenceFrame));

    MapConfig mapConfig;

    mapConfig.referenceFrame = referenceFrame;
    mapConfig.srs = registry::listSrs(mapConfig.referenceFrame);

    for (const auto &tilesetId : properties.tilesets) {
        mapConfig.mergeTileSet
            (TileSet::mapConfig
             (storage_paths::tilesetPath(root, tilesetId))
             , storage_paths::tilesetRoot());
    }

    for (const auto &item : properties.glues) {
        const auto &glue(item.second);
        mapConfig.mergeGlue
            (TileSet::mapConfig
             (storage_paths::gluePath(root, glue))
             , glue, storage_paths::glueRoot());
    }

    return mapConfig;
}

bool Storage::check(const boost::filesystem::path &root)
{
    try {
        Detail::loadConfig(root);
    } catch (const vadstena::storage::Error&) {
        return false;
    }
    return true;
}

bool Storage::externallyChanged() const
{
    return detail().externallyChanged();
}

vadstena::storage::Resources Storage::resources() const
{
    return {};
}

bool Storage::Detail::externallyChanged() const
{
    return (rootStat.changed(FileStat::stat(root))
            || configStat.changed(FileStat::stat(configPath))
            || extraConfigStat.changed(FileStat::stat
                                       (extraConfigPath, std::nothrow)));
}

std::time_t Storage::lastModified() const
{
    return detail().lastModified;
}

} } // namespace vadstena::vts
