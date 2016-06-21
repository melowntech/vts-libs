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

Storage::Detail::Detail(const boost::filesystem::path &iroot
                        , const StorageProperties &properties
                        , CreateMode mode)
    : root(fs::absolute(iroot))
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
        LOGTHROW(err1, vadstena::storage::Error)
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

ExtraStorageProperties Storage::Detail::loadExtraConfig() const
{
    return loadExtraConfig(root);
}

ExtraStorageProperties Storage::Detail::loadExtraConfig(const fs::path &root)
{
    try {
        // load config
        const auto p(storage::loadExtraConfig(root / ExtraConfigFilename));
        return p;
    } catch (const std::exception &e) {
        LOGTHROW(err2, vadstena::storage::Error)
            << "Unable to read extra config: <" << e.what() << ">.";
    }
    throw;
}

bool Glue::references(const std::string &tilesetId) const
{
    return (std::find(id.begin(), id.end(), tilesetId) != id.end());
}

StoredTileset::list::iterator
Storage::Properties::findTilesetIt(const TilesetId &tilesetId)
{
    return std::find_if(tilesets.begin(), tilesets.end()
                        , [&tilesetId](const StoredTileset &tileset)
                        { return tileset.tilesetId == tilesetId; });
}

StoredTileset::list::const_iterator
Storage::Properties::findTilesetIt(const TilesetId &tilesetId) const
{
    return std::find_if(tilesets.begin(), tilesets.end()
                        , [&tilesetId](const StoredTileset &tileset)
                        { return tileset.tilesetId == tilesetId; });
}

StoredTileset* Storage::Properties::findTileset(const TilesetId &tilesetId)
{
    auto it(findTilesetIt(tilesetId));
    if (it == tilesets.end()) { return nullptr; }
    return &*it;
}

const StoredTileset*
Storage::Properties::findTileset(const TilesetId &tilesetId)
    const
{
    auto it(findTilesetIt(tilesetId));
    if (it == tilesets.end()) { return nullptr; }
    return &*it;
}

int Storage::Properties::lastVersion(const TilesetId &baseId) const
{
    int version(-1);
    for (auto &id : tilesets) {
        if ((id.baseId == baseId) && (id.version > version)) {
            version = id.version;
        }
    }
    return version;
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
    TilesetIdList list;
    for (const auto &stored : detail().properties.tilesets) {
        list.push_back(stored.tilesetId);
    }
    return list;
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
    return mapConfig(path, loadConfig(path), loadExtraConfig(path));
}

MapConfig Storage::Detail::mapConfig() const
{
    return mapConfig(root, properties, loadExtraConfig());
}

namespace {

inline bool allowed(const TilesetIdSet *subset, const TilesetId &id)
{
    if (!subset) { return true; }
    if (!subset->count(id)) { return false; }
    return true;
}

inline bool allowed(const TilesetIdSet *subset, const Glue::Id &id)
{
    if (!subset) { return true; }
    for (const auto &tileset : id) {
        if (!subset->count(tileset)) { return false; }
    }
    return true;
}

inline bool allowed(const TilesetIdSet &subset, const Glue::Id &id)
{
    for (const auto &tileset : id) {
        if (!subset.count(tileset)) { return false; }
    }
    return true;
}

} // namespace

TilesetIdSet Storage::Properties::unique(const TilesetIdSet *subset) const
{
    typedef std::map<TilesetId, const StoredTileset*> Seen;
    Seen seen;

    for (const auto &tileset : tilesets) {
        // filter by set of allowed tilesets
        if (!allowed(subset, tileset.tilesetId)) { continue; }

        auto fseen(seen.find(tileset.baseId));
        if (fseen != seen.end()) {
            // baseId already seen -> check version and replace if better
            auto &current(fseen->second);
            if (tileset.version > current->version) {
                current = &tileset;
            }
        } else {
            // new, insert and remember
            seen.insert(Seen::value_type(tileset.baseId, &tileset));
        }
    }

    // reconstruct set from seen map
    TilesetIdSet out;
    for (const auto &item : seen) {
        out.insert(item.second->tilesetId);
    }

    return out;
}

MapConfig Storage::Detail::mapConfig(const boost::filesystem::path &root
                                     , const Storage::Properties &properties
                                     , const ExtraStorageProperties &extra
                                     , const TilesetIdSet *subset
                                     , const fs::path &prefix)
{
    auto referenceFrame(registry::Registry::referenceFrame
                        (properties.referenceFrame));

    MapConfig mapConfig;

    mapConfig.referenceFrame = referenceFrame;
    mapConfig.srs = registry::listSrs(mapConfig.referenceFrame);

    // prefill extra configuration
    mapConfig.credits = extra.credits;
    mapConfig.boundLayers = extra.boundLayers;

    // get in mapconfigs of tilesets and their glues; do not use any tileset's
    // extra configuration

    // grab list of unique tilesets to be sent into the output
    const auto unique(properties.unique(subset));

    // tilesets
    for (const auto &tileset : unique) {
        mapConfig.mergeTileSet
            (TileSet::mapConfig
             (storage_paths::tilesetPath(root, tileset), false)
             , prefix / storage_paths::tilesetRoot() / tileset);
    }

    // glues
    for (const auto &item : properties.glues) {
        // limit to tileset subset
        if (!allowed(unique, item.first)) { continue; }

        const auto &glue(item.second);
        mapConfig.mergeGlue
            (TileSet::mapConfig
             (storage_paths::gluePath(root, glue), false)
             , glue, prefix / storage_paths::glueRoot());
    }

    if (extra.position) {
        mapConfig.position = *extra.position;
    }

    mapConfig.rois = extra.rois;
    mapConfig.namedViews = extra.namedViews;

    if (extra.view) {
        // use settings from extra config
        mapConfig.view = extra.view;
    }

    // browser setup if present
    mapConfig.browserOptions = extra.browserOptions;

    return mapConfig;
}

MapConfig Storage::mapConfig(const boost::filesystem::path &root
                             , const ExtraStorageProperties &extra
                             , const TilesetIdSet &subset
                             , const fs::path &prefix)
{
    return Detail::mapConfig(root, Detail::loadConfig(root), extra, &subset
                             , prefix);
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

TileSet Storage::Detail::open(const TilesetId &tilesetId) const
{
    if (!properties.hasTileset(tilesetId)) {
        LOGTHROW(err1, vadstena::storage::NoSuchTileSet)
            << "Tileset <" << tilesetId << "> not found in storage "
            << root << ".";
    }

    return openTileSet(storage_paths::tilesetPath(root, tilesetId));
}

TileSet Storage::open(const TilesetId &tilesetId) const
{
    return detail().open(tilesetId);
}

TileSet Storage::Detail::open(const Glue &glue) const
{
    if (!properties.hasGlue(glue.id)) {
        LOGTHROW(err1, vadstena::storage::NoSuchTileSet)
            << "Glue <" << utility::join(glue.id, ",")
            << "> not found in storage "
            << root << ".";
    }

    return openTileSet(storage_paths::gluePath(root, glue));
}

TileSet Storage::open(const Glue &glue) const
{
    return detail().open(glue);
}

std::time_t Storage::lastModified() const
{
    return detail().lastModified;
}

boost::filesystem::path Storage::path() const
{
    return detail().root;
}

const StorageProperties& Storage::getProperties() const
{
    return detail().properties;
}

const registry::ReferenceFrame& Storage::referenceFrame() const
{
    return detail().referenceFrame;
}

TilesetIdList tilesetIdList(const StoredTileset::list &tilesets)
{
    TilesetIdList list;
    for (const auto &ts : tilesets) { list.push_back(ts.tilesetId); }
    return list;
}

GlueIndices buildGlueIndices(const TilesetIdList &world, const Glue::Id &id)
{
    GlueIndices indices;
    std::size_t i(0);
    for (const auto &ts : id) {
        if (i >= world.size()) {
            LOGTHROW(err2, vadstena::storage::Error)
                << "Glue <" << utility::join(id, ", ")
                << "> doesn't belong into world <"
                << utility::join(world, ", ") << ">.";
        }

        while (i < world.size()) {
            if (world[i] == ts) {
                indices.push_back(i);
                ++i;
                break;
            } else {
                ++i;
            }
        }
    }
    return indices;
}

Glue::list Storage::glues(const TilesetId &tilesetId) const
{
    Glue::list glues;

    for (const auto &item : detail().properties.glues) {
        const auto &glue(item.second);
        if (glue.id.back() == tilesetId) {
            glues.push_back(glue);
        }
    }

    return glues;
}

Glue::list
Storage::glues(const TilesetId &tilesetId
               , const std::function<bool(const Glue::Id&)> &filter) const
{
    Glue::list glues;

    for (const auto &item : detail().properties.glues) {
        const auto &glue(item.second);
        if (glue.id.back() == tilesetId) {
            if (!filter(glue.id)) { continue; }
            glues.push_back(glue);
        }
    }

    return glues;
}

boost::filesystem::path Storage::path(const TilesetId &tilesetId) const
{
    const auto &root(detail().root);
    if (!detail().properties.hasTileset(tilesetId)) {
        LOGTHROW(err1, vadstena::storage::NoSuchTileSet)
            << "Tileset <" << tilesetId << "> not found in storage "
            << root << ".";
    }

    return storage_paths::tilesetPath(root, tilesetId);
}

boost::filesystem::path Storage::path(const Glue &glue) const
{
    const auto &root(detail().root);
    if (!detail().properties.hasGlue(glue.id)) {
        LOGTHROW(err1, vadstena::storage::NoSuchTileSet)
            << "Glue <" << utility::join(glue.id, ",")
            << "> not found in storage "
            << root << ".";
    }

    return storage_paths::gluePath(root, glue);
}

TileSet Storage::flatten(const boost::filesystem::path &tilesetPath
                         , CreateMode mode
                         , const boost::optional<std::string> &tilesetId)
{
    CloneOptions co;

    // create in-memory
    co.tilesetId(TilesetId("storage"));
    auto tmp(aggregateTileSets(*this, co, tilesets()));

    // clone
    co.mode(mode);
    co.sameType(false);
    co.tilesetId(tilesetId ? *tilesetId : tilesetPath.filename().string());
    return cloneTileSet(tilesetPath, tmp, co);
}

} } // namespace vadstena::vts
