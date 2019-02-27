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
#include <boost/utility/in_place_factory.hpp>
#include <boost/noncopyable.hpp>
#include <boost/filesystem.hpp>
#include <boost/format.hpp>
#include <boost/range/adaptor/reversed.hpp>

#include "utility/streams.hpp"
#include "utility/guarded-call.hpp"
#include "utility/path.hpp"

#include "../../storage/error.hpp"
#include "../storage.hpp"
#include "../../vts.hpp"
#include "detail.hpp"
#include "../tileset/detail.hpp"
#include "../config.hpp"

#include "config.hpp"
#include "paths.hpp"

namespace fs = boost::filesystem;

namespace vtslibs { namespace vts {

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
                      , CreateMode mode, const StorageLocker::pointer &locker)
{
    return { path, properties, mode, locker };
}

Storage openStorage(const boost::filesystem::path &path
                    , OpenMode mode
                    , const StorageLocker::pointer &locker)
{
    return { path, mode, locker };
}

Storage::Storage(const boost::filesystem::path &path, OpenMode mode
                 , const StorageLocker::pointer &locker)
    : detail_(std::make_shared<Detail>(path, mode, locker))
{
}

Storage::Storage(const boost::filesystem::path &path
                 , const StorageProperties &properties
                 , CreateMode mode
                 , const StorageLocker::pointer &locker)
    : detail_(std::make_shared<Detail>(path, properties, mode, locker))
{}

Storage::~Storage()
{
    // no-op
}

Storage::Detail::~Detail()
{
}

Storage::Detail::Detail(const boost::filesystem::path &iroot
                        , const StorageProperties &properties, CreateMode mode
                        , const StorageLocker::pointer &locker)
    : storageLock(locker)
    , root(fs::absolute(iroot))
    , configPath(root / ConfigFilename)
    , extraConfigPath(root / ExtraConfigFilename)
    , referenceFrame(registry::system.referenceFrames
                     (properties.referenceFrame))
{
    // fill in slice
    static_cast<StorageProperties&>(this->properties) = properties;

    if (!create_directories(root)) {
        // directory already exists -> fail if mode says so
        if (mode == CreateMode::failIfExists) {
            LOGTHROW(err2, vtslibs::storage::StorageAlreadyExists)
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

Storage::Detail::Detail(const boost::filesystem::path &root, OpenMode mode
                        , const StorageLocker::pointer &locker)
    : storageLock(locker)
    , root(root)
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

    referenceFrame = registry::system.referenceFrames
        (properties.referenceFrame);
}

void Storage::Detail::loadConfig()
{
    properties = loadConfig(root);
}

Storage::Properties Storage::Detail::readConfig()
{
    return loadConfig(root);
}

Storage::Properties Storage::Detail::loadConfig(const fs::path &root)
{
    try {
        // load config
        const auto p(storage::loadConfig(root / ConfigFilename));
        return p;
    } catch (const std::exception &e) {
        LOGTHROW(err1, vtslibs::storage::Error)
            << "Unable to read config: <" << e.what() << ">.";
    }
    throw;
}

void Storage::Detail::saveConfig()
{
    // save json
    try {
        ConfigFileGuard tmpFile(configPath);
        storage::saveConfig(tmpFile.path(), properties);
    } catch (const std::exception &e) {
        LOGTHROW(err2, vtslibs::storage::Error)
            << "Unable to write config " << configPath
            << ": <" << e.what() << ">.";
    }
}

void Storage::Detail::saveConfig(const Properties &nProperties)
{
    properties = nProperties;
    saveConfig();
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
        LOGTHROW(err2, vtslibs::storage::Error)
            << "Unable to read extra config: <" << e.what() << ">.";
    }
    throw;
}

bool Glue::references(const std::string &tilesetId) const
{
    return (std::find(id.begin(), id.end(), tilesetId) != id.end());
}

bool Glue::references(const Glue::Id &id, const std::string &tilesetId)
{
    return (std::find(id.begin(), id.end(), tilesetId) != id.end());
}

bool VirtualSurface::references(const std::string &tilesetId) const
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

VirtualSurface::map::iterator Storage::Properties
::findVirtualSurface(const VirtualSurface::Id &virtualSurface)
{
    return virtualSurfaces.find(virtualSurface);
}

VirtualSurface::map::const_iterator Storage::Properties
::findVirtualSurface(const VirtualSurface::Id& virtualSurface) const
{
    return virtualSurfaces.find(virtualSurface);
}

Glue::Id Storage::Properties::normalize(const Glue::Id &id) const
{
    Glue::Id out;
    out.reserve(id.size());

    const TilesetIdSet tmp(id.begin(), id.end());

    for (const auto &tileset : tilesets) {
        if (tmp.find(tileset.tilesetId) != tmp.end()) {
            out.push_back(tileset.tilesetId);
        }
    }

    return out;
}

TilesetIdList Storage::tilesets() const
{
    TilesetIdList list;
    for (const auto &stored : detail().properties.tilesets) {
        list.push_back(stored.tilesetId);
    }
    return list;
}

TilesetIdList Storage::tilesets(const TilesetIdSet &subset) const
{
    TilesetIdList list;
    for (const auto &stored : detail().properties.tilesets) {
        if (subset.count(stored.tilesetId)) {
            list.push_back(stored.tilesetId);
        }
    }
    return list;
}

StoredTileset::list Storage::storedTilesets() const {
    return detail().properties.tilesets;
}

Proxy2ExternalUrl Storage::gluesExternalUrl() const
{
    return detail().properties.gluesExternalUrl;
}

Proxy2ExternalUrl Storage::vsExternalUrl() const
{
    return detail().properties.vsExternalUrl;
}

Glue::map Storage::glues() const
{
    return detail().properties.glues;
}

VirtualSurface::map Storage::virtualSurfaces() const
{
    return detail().properties.virtualSurfaces;
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

/** Helper type to hold exact tileset ids and base tileset ids with tags
 */
struct TilesetSubset {
    typedef boost::optional<TilesetSubset> optional;

    typedef std::map<TilesetId, std::string> TaggedIds;

    TilesetIdSet ids;
    TaggedIds taggedIds;
};

TilesetSubset::optional makeSubsetFilter(const TilesetIdSet *subset)
{
    if (!subset) { return boost::none; }

    boost::optional<TilesetSubset> res;
    res = boost::in_place();
    auto &filter(*res);

    for (const auto &tid : *subset) {
        auto hash(tid.find('#'));
        if (hash == std::string::npos) {
            // no @ character
            filter.ids.insert(tid);
            continue;
        }

        // tagged
        filter.taggedIds.insert
            (TilesetSubset::TaggedIds::value_type
             (tid.substr(0, hash), tid.substr(hash + 1)));
    }

    return filter;
}

inline TilesetIdSet asSet(const TilesetIdMap &map) {
    TilesetIdSet set;
    for (const auto &pair : map) { set.insert(pair.first); }
    return set;
}

inline bool allowed(const TilesetIdSet *subset, const TilesetId &id)
{
    if (!subset) { return true; }
    return subset->count(id);
}

enum class Allowed { false_ = false, true_ = true, tagged = 2 };

inline Allowed allowed(const TilesetSubset::optional &ofilter
                       , const StoredTileset &storedTs)
{
    if (!ofilter) { return Allowed::true_; }
    const auto &filter(*ofilter);

    // check exact match
    if (filter.ids.count(storedTs.tilesetId)) { return Allowed::true_; }

    // check for tag via tileset's baseId
    auto ftagged(filter.taggedIds.find(storedTs.baseId));
    if (ftagged == filter.taggedIds.end()) {
        // no such baseId
        return Allowed::false_;
    }

    // check for tag presence
    return (storedTs.tags.count(ftagged->second)
            ? Allowed::tagged : Allowed::false_);
}

inline bool allowed(const TilesetIdSet *subset, const Glue::Id &id)
{
    if (!subset) { return true; }
    for (const auto &tileset : id) {
        if (!subset->count(tileset)) { return false; }
    }
    return true;
}

inline bool allowed(const TilesetIdSet &subset, const TilesetId &id)
{
    return subset.count(id);
}

inline bool allowed(const TilesetIdMap &subset, const TilesetId &id)
{
    return subset.count(id);
}

inline bool allowed(const TilesetIdSet &subset, const Glue::Id &id)
{
    for (const auto &tileset : id) {
        if (!subset.count(tileset)) { return false; }
    }
    return true;
}

inline bool allowed(const TilesetIdMap &subset, const Glue::Id &id)
{
    for (const auto &tileset : id) {
        if (!subset.count(tileset)) { return false; }
    }
    return true;
}

/** Checks if there is any real rename in the provided tileset ID map. Returns
 *  the pointer to the provided map if any difference is encoutered, null
 *  otherwise.
 */
inline const TilesetIdMap* optionalRename(const TilesetIdMap &map)
{
    for (const auto &pair : map) {
        if (pair.first != pair.second) { return &map; }
    }
    return nullptr;
}

} // namespace

TilesetIdMap Storage::Properties::unique(const TilesetIdSet *subset) const
{
    const auto filter(makeSubsetFilter(subset));

    struct Seen {
        const StoredTileset *tileset;
        TilesetId tilesetId;

        Seen(const StoredTileset *tileset, const TilesetId &tilesetId)
            : tileset(tileset), tilesetId(tilesetId)
        {}

        typedef std::map<TilesetId, Seen> map;
    };

    Seen::map seen;

    for (const auto &tileset : tilesets) {
        // filter by set of allowed tilesets
        TilesetId useTilesetId(tileset.tilesetId);

        switch (allowed(filter, tileset)) {
        case Allowed::false_: continue;
        case Allowed::true_: break;
        case Allowed::tagged:
            // tagged version, rename to baseId
            useTilesetId = tileset.baseId; break;
        }

        auto fseen(seen.find(tileset.baseId));
        if (fseen != seen.end()) {
            // baseId already seen -> check version and replace if better
            auto &current(fseen->second);
            if (tileset.version > current.tileset->version) {
                current.tileset = &tileset;
                current.tilesetId = useTilesetId;
            }
        } else {
            // new, insert and remember
            seen.insert(Seen::map::value_type
                        (tileset.baseId, Seen(&tileset, useTilesetId)));
        }
    }

    // reconstruct set from seen map
    TilesetIdMap out;
    for (const auto &item : seen) {
        out.insert(TilesetIdMap::value_type
                   (item.second.tileset->tilesetId, item.second.tilesetId));
    }

    return out;
}

TilesetIdCounts Storage::Properties::pendingGlueCount(TilesetIdSet tilesets)
    const
{
    // preinitialize
    TilesetIdCounts counts;
    for (const auto &ts : tilesets) {
        counts.insert(TilesetIdCounts::value_type(ts, 0));
    }

    for (const auto &glue : pendingGlues) {
        if (!allowed(tilesets, glue)) { continue; }
        auto fcounts(counts.find(glue.back()));
        if (fcounts != counts.end()) {
            ++fcounts->second;
        }
    }

    return counts;
}

Glue::IdSet Storage::Detail::pendingGlues(const TilesetIdSet *subset) const
{
    // grab list of unique tilesets to be sent into the output
    const auto unique(properties.unique(subset));

    Glue::IdSet pending;

    for (const auto &glueId : properties.pendingGlues) {
        bool add(true);
        for (const auto &tilesetId : glueId) {
            // set add to result of check for tileset in unique set of tilesets
            if (!(add = unique.count(tilesetId))) {
                // not found -> stop checking glue ID now
                break;
            }
        }

        if (add) {
            // all check passed -> remember
            pending.insert(glueId);
        }
    }

    return pending;
}

MapConfig Storage::Detail::mapConfig(const boost::filesystem::path &root
                                     , const Storage::Properties &properties
                                     , const ExtraStorageProperties &extra
                                     , const TilesetIdSet *subset
                                     , const TilesetIdSet *freeLayers
                                     , const fs::path &prefix)
{
    auto referenceFrame(registry::system.referenceFrames
                        (properties.referenceFrame));

    MapConfig mapConfig;

    mapConfig.referenceFrame = referenceFrame;
    mapConfig.srs = registry::listSrs(mapConfig.referenceFrame);

    // prefill extra configuration
    mapConfig.credits = extra.credits;
    mapConfig.boundLayers = extra.boundLayers;
    mapConfig.freeLayers = extra.freeLayers;
    mapConfig.bodies = extra.bodies;

    // get in mapconfigs of tilesets and their glues; do not use any tileset's
    // extra configuration

    // grab list of unique tilesets (well, it is not a list but a mapping
    // between real tilesetId and tilesetId to be used in the output)
    const auto unique(properties.unique(subset));

    // convert unique map above to optional tilesetId renamer; pointer is null
    // if there is no id change at all.
    const auto *tilesetRename(optionalRename(unique));

    // // counts of pending glues for unique tilesets
    // const auto pgc(properties.pendingGlueCount(unique));

    // set of tilesets with glues
    TilesetIdSet glueable;

    const auto tilesetUrl([&](const StoredTileset &tileset) -> SurfaceRoot
    {
        // synthetic default URL
        const auto defaultPath
            (prefix / storage_paths::tilesetRoot() / tileset.tilesetId);

        // no per-url configuration, return default
        if (tileset.proxy2ExternalUrl.empty()) { return defaultPath; }

        return SurfaceRoot([tileset, defaultPath](const OProxy &proxy)
                           -> fs::path
        {
            if (proxy) {
                const auto &f(tileset.proxy2ExternalUrl.find(*proxy));
                if (f != tileset.proxy2ExternalUrl.end()) { return f->second; }
            }

            // not found -> default
            return defaultPath;
        });
    });

    const auto supportTilesetUrl([&](const Proxy2ExternalUrl &externalUrl
                                     , const fs::path &defaultRoot)
                                 -> SurfaceRoot
    {
        // synthetic default URL
        const auto defaultPath(prefix / defaultRoot);

        // no per-url configuration, return default
        if (externalUrl.empty()) { return defaultPath; }

        return SurfaceRoot([externalUrl, defaultPath](const OProxy &proxy)
                           -> fs::path
        {
            if (proxy) {
                const auto &f(externalUrl.find(*proxy));
                if (f != externalUrl.end()) { return f->second; }
            }

            // not found -> default
            return defaultPath;
        });
    });

    // tilesets
    // bool surfacesAvailable(true);
    for (const auto &tileset : properties.tilesets) {
        // check for tileset being both requested and fully available
        bool surface(allowed(unique, tileset.tilesetId));

        // should we use it as a free layer?
        bool fl(freeLayers && freeLayers->count(tileset.tilesetId));

        // handle tileset as a free layers
        if (fl) {
            // tileset path as a root
            mapConfig.addMeshTilesConfig
                (TileSet::meshTilesConfig
                 (storage_paths::tilesetPath(root, tileset.tilesetId), false)
                 , tilesetUrl(tileset), tilesetRename);
        }

        // handle tileset as a surface
        if (surface) {
            glueable.insert(tileset.tilesetId);
            mapConfig.mergeTileSet
                (TileSet::mapConfig
                 (storage_paths::tilesetPath(root, tileset.tilesetId), false)
                 , tilesetUrl(tileset), tilesetRename);
        }
    }

    // glues
    for (const auto &item : properties.glues) {
        // limit to tileset subset
        if (!allowed(glueable, item.first)) { continue; }

        const auto &glue(item.second);
        mapConfig.mergeGlue
            (TileSet::mapConfig
             (storage_paths::gluePath(root, glue), false)
             , glue, supportTilesetUrl
             (properties.gluesExternalUrl, storage_paths::glueRoot())
             , tilesetRename);
    }

    // virtualSurfaces
    if (extra.virtualSurfacesEnabled) {
        for (const auto &item : properties.virtualSurfaces) {
            // limit to tileset subset
            if (!allowed(unique, item.first)) { continue; }

            const auto &virtualSurface(item.second);
            mapConfig.mergeVirtualSurface
                (TileSet::mapConfig
                 (storage_paths::virtualSurfacePath
                  (root, virtualSurface), false)
                 , virtualSurface
                 , supportTilesetUrl(properties.vsExternalUrl
                                     , storage_paths::virtualSurfaceRoot())
                 , tilesetRename);
        }
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
                             , const TilesetIdSet &freeLayers
                             , const fs::path &prefix)
{
    return Detail::mapConfig(root, Detail::loadConfig(root)
                             , extra, &subset, &freeLayers, prefix);
}

bool Storage::check(const boost::filesystem::path &root)
{
    // try config existence
    try {
        return fs::exists(root / ConfigFilename);
    } catch (const fs::filesystem_error&) {}
    return false;
}

bool Storage::check(const boost::filesystem::path &root
                    , const std::string &mime)
{
    if (mime == "inode/directory") { return check(root); }
    return false;
}

bool Storage::externallyChanged() const
{
    return detail().externallyChanged();
}

vtslibs::storage::Resources Storage::resources() const
{
    return {};
}

bool Storage::Detail::externallyChanged() const
{
    return (rootStat.changed(FileStat::stat(root, std::nothrow))
            || configStat.changed(FileStat::stat(configPath, std::nothrow))
            || extraConfigStat.changed(FileStat::stat
                                       (extraConfigPath, std::nothrow)));
}

TileSet Storage::Detail::open(const TilesetId &tilesetId) const
{
    if (!properties.hasTileset(tilesetId)) {
        LOGTHROW(err1, vtslibs::storage::NoSuchTileSet)
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
        LOGTHROW(err1, vtslibs::storage::NoSuchTileSet)
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
            LOGTHROW(err2, vtslibs::storage::Error)
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

Glue::IdSet Storage::pendingGlues() const
{
    return detail().properties.pendingGlues;
}

Glue::IdSet Storage::pendingGlues(const TilesetId &tilesetId) const
{
    Glue::IdSet glues;

    const auto &src(detail().properties.pendingGlues);
    std::copy_if(src.begin(), src.end(), std::inserter(glues, glues.end())
                 , [&](const Glue::Id &id)
                 {
                     return Glue::references(id, tilesetId);
                 });

    return glues;
}

Glue::IdSet Storage::pendingGlues(const TilesetIdSet *subset) const
{
    return detail().pendingGlues(subset);
}

VirtualSurface::list Storage::virtualSurfaces(const TilesetId &tilesetId) const
{
    VirtualSurface::list virtualSurfaces;

    for (const auto &item : detail().properties.virtualSurfaces) {
        const auto &virtualSurface(item.second);
        if (virtualSurface.id.back() == tilesetId) {
            virtualSurfaces.push_back(virtualSurface);
        }
    }

    return virtualSurfaces;
}

VirtualSurface::list Storage
::virtualSurfaces(const TilesetId &tilesetId
                  , const std::function<bool(const VirtualSurface::Id&)>
                  &filter) const
{
    VirtualSurface::list virtualSurfaces;

    for (const auto &item : detail().properties.virtualSurfaces) {
        const auto &virtualSurface(item.second);
        if (virtualSurface.id.back() == tilesetId) {
            if (!filter(virtualSurface.id)) { continue; }
            virtualSurfaces.push_back(virtualSurface);
        }
    }

    return virtualSurfaces;
}

boost::filesystem::path Storage::path(const TilesetId &tilesetId) const
{
    const auto &root(detail().root);
    if (!detail().properties.hasTileset(tilesetId)) {
        LOGTHROW(err1, vtslibs::storage::NoSuchTileSet)
            << "Tileset <" << tilesetId << "> not found in storage "
            << root << ".";
    }

    return storage_paths::tilesetPath(root, tilesetId);
}

boost::filesystem::path Storage::path(const Glue &glue) const
{
    const auto &root(detail().root);
    if (const auto *g = detail().properties.getGlue(glue.id)) {
        return storage_paths::gluePath(root, *g);
    }

    LOGTHROW(err1, vtslibs::storage::NoSuchTileSet)
        << "Glue <" << utility::join(glue.id, ",")
        << "> not found in storage "
        << root << ".";
    throw;
}

boost::filesystem::path Storage::path(const Glue::Id &glueId) const
{
    const auto &root(detail().root);
    if (const auto *g = detail().properties.getGlue(glueId)) {
        return storage_paths::gluePath(root, *g);
    }

    LOGTHROW(err1, vtslibs::storage::NoSuchTileSet)
        << "Glue <" << utility::join(glueId, ",")
        << "> not found in storage "
        << root << ".";
    throw;
}

boost::filesystem::path Storage::path(const VirtualSurface &virtualSurface)
    const
{
    const auto &root(detail().root);
    if (const auto *vs
        = detail().properties.getVirtualSurface(virtualSurface.id)) {
        return storage_paths::virtualSurfacePath(root, *vs);
    }

    LOGTHROW(err1, vtslibs::storage::NoSuchTileSet)
        << "VirtualSurface <" << utility::join(virtualSurface.id, ",")
        << "> not found in storage "
        << root << ".";
    throw;
}

TileSet Storage::clone(const boost::filesystem::path &tilesetPath
                       , const CloneOptions &createOptions
                       , const TilesetIdSet *subset) const
{
    // create in-memory
    auto tmp(aggregateTileSets(*this, CloneOptions(createOptions)
                               .tilesetId(TilesetId("storage"))
                               , asSet(detail().properties.unique(subset))));

    // clone
    CloneOptions co(createOptions);
    co.sameType(false);
    if (!co.tilesetId()) { co.tilesetId(tilesetPath.filename().string()); }
    return cloneTileSet(tilesetPath, tmp, co);
}

void Storage::relocate(const boost::filesystem::path &root
                       , const RelocateOptions &ro
                       , const std::string &prefix)
{
    if (ro.dryRun) {
        LOG(info3) << prefix << "Simulating relocation of storage "
                   << root << ".";
    } else {
        LOG(info3) << prefix << "Relocating " << root << ".";
    }

    auto config(storage::loadConfig(root / ConfigFilename));

    for (const auto &tileset : config.tilesets) {
        TileSet::relocate(storage_paths::tilesetPath(root, tileset.tilesetId)
                          , ro, prefix + "    ");
    }
}

void Storage::reencode(const boost::filesystem::path &root
                       , const ReencodeOptions &ro
                       , const std::string &prefix)
{
    if (ro.cleanup) {
        if (ro.dryRun) {
            LOG(info3) << prefix << "Simulating reencode cleanup of storage "
                       << root << ".";
        } else {
            LOG(info3)
                << prefix << "Cleaning up reencode of storage " << root << ".";
        }
    } else {
        if (ro.dryRun) {
            LOG(info3) << prefix << "Simulating reencode of storage "
                       << root << ".";
        } else {
            LOG(info3) << prefix << "Reencode of storage " << root << ".";
        }
    }

    auto config(storage::loadConfig(root / ConfigFilename));

    for (const auto &tileset : config.tilesets) {
        TileSet::reencode(storage_paths::tilesetPath(root, tileset.tilesetId)
                          , ro, prefix + "    ");
    }

    for (const auto &glue : config.glues) {
        TileSet::reencode(storage_paths::gluePath(root, glue.second)
                          , ro, prefix + "    ");
    }

    // virtual surfaces: just version bump, do not descend down (imminent
    // infinite recursion)
    auto vsRo(ro);
    vsRo.descend = false;
    for (const auto &virtualSurface : config.virtualSurfaces) {
        TileSet::reencode(storage_paths::virtualSurfacePath
                          (root, virtualSurface.second)
                          , vsRo, prefix + "    ");
    }
}

void Storage::updateTags(const TilesetId &tilesetId
                         , const Tags &add, const Tags &remove)
{
    detail().updateTags(tilesetId, add, remove);
}

void Storage::Detail::updateTags(const TilesetId &tilesetId
                                 , const Tags &add, const Tags &remove)
{
    // (re)load config to have fresh copy when under lock
    if (storageLock) { loadConfig(); }

    auto *tileset(properties.findTileset(tilesetId));
    if (!tileset) {
        LOGTHROW(err1, vtslibs::storage::NoSuchTileSet)
            << "Tileset <" << tilesetId << "> not found in storage "
            << root << ".";
    }

    // add tags to be added
    tileset->tags.insert(add.begin(), add.end());

    // remove tags to be removed
    for (const auto &tag : remove) { tileset->tags.erase(tag); }

    // store config
    saveConfig();
}

void Storage::updateExternalUrl(const TilesetId &tilesetId
                                , const Proxy2ExternalUrl &add
                                , const std::vector<std::string> &remove)
{
    detail().updateExternalUrl(tilesetId, add, remove);
}

void Storage::Detail
::updateExternalUrl(const TilesetId &tilesetId
                    , const Proxy2ExternalUrl &add
                    , const std::vector<std::string> &remove)
{
    // (re)load config to have fresh copy when under lock
    if (storageLock) { loadConfig(); }

    Proxy2ExternalUrl *proxy2ExternalUrl(nullptr);

    if (tilesetId == "@glues") {
        proxy2ExternalUrl = &properties.gluesExternalUrl;
    } else if (tilesetId == "@vs") {
        proxy2ExternalUrl = &properties.vsExternalUrl;
    } else if (auto *tileset = properties.findTileset(tilesetId)) {
        proxy2ExternalUrl = &tileset->proxy2ExternalUrl;
    }

    if (!proxy2ExternalUrl) {
        LOGTHROW(err1, vtslibs::storage::NoSuchTileSet)
            << "Tileset <" << tilesetId << "> not found in storage "
            << root << ".";
    }

    // add external URLs
    for (const auto &item : add) {
        (*proxy2ExternalUrl)[item.first] = item.second;
    }

    // remove external URLs for given proxies
    for (const auto &proxy : remove) {
        proxy2ExternalUrl->erase(proxy);
    }

    // store config
    saveConfig();
}

std::tuple<VirtualSurface::Id, TileSet>
Storage::openVirtualSurface(const TilesetIdSet &tilesets) const
{
    return detail().openVirtualSurface(tilesets);
}

std::tuple<VirtualSurface::Id, TileSet>
Storage::Detail::openVirtualSurface(const TilesetIdSet &tilesets) const
{
    auto tmp(tilesets);

    VirtualSurface::Id vsId;
    for (const auto &stored : properties.tilesets) {
        auto ftmp(tmp.find(stored.tilesetId));
        if (ftmp == tmp.end()) { continue; }

        vsId.push_back(stored.tilesetId);
        tmp.erase(ftmp);
    }

    if (!tmp.empty()) {
        LOGTHROW(err1, vtslibs::storage::NoSuchTileSet)
            << "Tileset(s) <" << utility::join(tmp, ", ")
            << "> not found in storage " << root << ".";
    }

    const auto fvirtualSurfaces(properties.findVirtualSurface(vsId));
    if (fvirtualSurfaces == properties.virtualSurfaces.end()) {
        LOGTHROW(err1, vtslibs::storage::NoSuchTileSet)
            << "Virtual surface <"
            << utility::join(vsId, ",") << "> "
            "not found in storage " << root << ".";
    }

    return std::tuple<VirtualSurface::Id, TileSet>
        (vsId, openTileSet(storage_paths::virtualSurfacePath
                           (root, fvirtualSurfaces->second)));
}

} } // namespace vtslibs::vts
