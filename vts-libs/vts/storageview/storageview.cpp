/**
 * \file vts/storage/storageview.cpp
 * \author Vaclav Blazek <vaclav.blazek@citationtech.net>
 *
 * Tile set storage view access.
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
#include "../storageview.hpp"
#include "../../vts.hpp"
#include "./detail.hpp"
#include "../tileset/detail.hpp"

#include "./config.hpp"

namespace fs = boost::filesystem;

namespace utility {

inline fs::path cleanAbsolute(fs::path in)
{
    in = absolute(in);
    fs::path p;
    for (auto i(in.begin()), e(in.end()); i != e; ++i) {
        const auto &part(*i);
        if (part == ".") {
        } else if (part == "..") {
            p = p.parent_path();
        } else {
            p /= part;
        }
    }
    return p;
}

fs::path relpath(const fs::path &src, const fs::path &dst)
{
    auto s(cleanAbsolute(src));
    auto d(cleanAbsolute(dst));

    auto is(s.begin()), es(s.end());
    auto id(d.begin()), ed(d.end());

    for (; (is != es) && (id != ed); ++is, ++id) {
        if (*is != *id) { break; }
    }

    fs::path p;
    // append ".." for each source element
    for (; is != es; ++is) {
        p /= "..";
    }
    // copy whole destination
    for (; id != ed; ++id) {
        p /= *id;
    }

    return p;
}

} // namespace utility

namespace vadstena { namespace vts {

StorageView::StorageView(const boost::filesystem::path &path)
    : detail_(new Detail(path))
{
}

StorageView::~StorageView()
{
    // no-op
}

StorageView::Detail::~Detail()
{
}

StorageView::Detail::Detail(const boost::filesystem::path &root)
    : configPath(root)
    , properties(storageview::loadConfig(root))
    , configStat(FileStat::stat(configPath))
    , lastModified(configStat.lastModified)
    , storage(properties.storagePath, OpenMode::readOnly)
{}

void StorageView::Detail::loadConfig()
{
    properties = loadConfig(configPath);
}

StorageView::Properties StorageView::Detail::loadConfig(const fs::path &path)
{
    try {
        // load config
        return storageview::loadConfig(path);
    } catch (const std::exception &e) {
        LOGTHROW(err1, vadstena::storage::Error)
            << "Unable to read config: <" << e.what() << ">.";
    }
    throw;
}

MapConfig StorageView::mapConfig() const
{
    return detail().mapConfig();
}

MapConfig StorageView::mapConfig(const boost::filesystem::path &configPath)

{
    return Detail::mapConfig(configPath, Detail::loadConfig(configPath));
}

MapConfig StorageView::Detail::mapConfig() const
{
    return mapConfig(configPath, properties);
}

MapConfig
StorageView::Detail::mapConfig(const boost::filesystem::path &configPath
                               , const StorageView::Properties &properties)
{
    // NB: view behaves like a directory although it is a single file
    return Storage::mapConfig
        (properties.storagePath, properties.extra
         , properties.tilesets
         , utility::relpath(configPath, properties.storagePath));
}

bool StorageView::check(const boost::filesystem::path &path)
{
    try {
        Detail::loadConfig(path);
    } catch (const vadstena::storage::Error&) {
        return false;
    }
    return true;
}

bool StorageView::externallyChanged() const
{
    return detail().externallyChanged();
}

vadstena::storage::Resources StorageView::resources() const
{
    return {};
}

bool StorageView::Detail::externallyChanged() const
{
    return (configStat.changed(FileStat::stat(configPath))
            || storage.externallyChanged());
}

std::time_t StorageView::lastModified() const
{
    return detail().lastModified;
}

const Storage& StorageView::storage() const
{
    return detail().storage;
}

const TilesetIdSet& StorageView::tilesets() const
{
    return detail().properties.tilesets;
}

StorageView openStorageView(const boost::filesystem::path &path)
{
    return { path };
}

} } // namespace vadstena::vts
