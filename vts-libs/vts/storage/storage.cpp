/**
 * \file vts/storage/storage.cpp
 * \author Vaclav Blazek <vaclav.blazek@citationtech.net>
 *
 * Tile set storage access.
 */

#include <memory>
#include <string>
#include <exception>

#include <boost/optional.hpp>
#include <boost/noncopyable.hpp>
#include <boost/filesystem.hpp>
#include <boost/format.hpp>

#include "utility/streams.hpp"
#include "utility/guarded-call.hpp"

#include "../../storage/error.hpp"
#include "../storage.hpp"
#include "../../vts.hpp"
#include "./detail.hpp"
#include "./config.hpp"

namespace fs = boost::filesystem;

namespace vadstena { namespace vts {

namespace {
    const fs::path ConfigFilename("storage.conf");
    const fs::path TileSetDir("tilesets");
    const fs::path TempDir("tmp");
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

void Storage::add(const boost::filesystem::path &tilesetPath
                  , const Location &where, CreateMode mode
                  , const boost::optional<std::string> tilesetId)
{
    auto ts(openTileSet(tilesetPath));
    detail().add(ts, where, mode
                 , tilesetId ? *tilesetId : ts.getProperties().id);
}

Storage::Detail::~Detail()
{
}

Storage::Detail::Detail(const boost::filesystem::path &root
                        , const StorageProperties &properties
                        , CreateMode mode)
    : root(root)
{
    // fill in slice
    static_cast<StorageProperties&>(this->properties) = properties;

    if (!create_directories(root)) {
        // directory already exists -> fail if mode says so
        if (mode == CreateMode::failIfExists) {
            LOGTHROW(err2, vadstena::storage::StorageAlreadyExists)
                << "Storage at " << root << " already exists.";
        }

        // OK, we can overwrite; cache contents of old config (if any)
        try {
            auto old(storage::loadConfig(root / ConfigFilename));
            this->properties.revision = old.revision + 1;
        } catch (...) {}
    }

    saveConfig();
}

Storage::Detail::Detail(const boost::filesystem::path &root
                        , OpenMode mode)
    : root(root)
{
    (void) mode;

    loadConfig();
}

void Storage::Detail::loadConfig()
{
    try {
        // load config
        const auto p(storage::loadConfig(root / ConfigFilename));

        // set
        properties = p;
    } catch (const std::exception &e) {
        LOGTHROW(err2, vadstena::storage::Error)
            << "Unable to read config: <" << e.what() << ">.";
    }
}

void Storage::Detail::saveConfig()
{
    // save json
    try {
        storage::saveConfig(root / ConfigFilename, properties);
    } catch (const std::exception &e) {
        LOGTHROW(err2, vadstena::storage::Error)
            << "Unable to write config: <" << e.what() << ">.";
    }
}

namespace {

class Tx {
public:
    Tx() {}
    ~Tx();

    void add(const fs::path &work, const fs::path &dst);

private:
    void rollback();
    void commit();

    typedef std::map<fs::path, fs::path> Mapping;
    Mapping mapping_;
};

Tx::~Tx() {
    if (std::uncaught_exception()) {
        // we cannot throw!
        rollback();
    } else {
        commit();
    }
}

void Tx::add(const fs::path &work, const fs::path &dst)
{
    mapping_.insert(Mapping::value_type(work, dst));
}

void Tx::rollback()
{
    for (const auto &item : mapping_) {
        try { remove_all(item.first); } catch (...) {}
    }
}

void Tx::commit()
{
    for (const auto &item : mapping_) {
        // TODO: make more robust
        // remove old stuff
        remove_all(item.second);
        // move new stuff there
        rename(item.first, item.second);
    }
}

} // namespace

    /** Returns list of tileses in the stacked order (bottom to top);
     */
std::vector<std::string> Storage::tilesets() const
{
    return detail().properties.tilesets;
}

Glue::list Storage::glues() const
{
    return detail().properties.glues;
}

void Storage::Detail::add(const TileSet &tileset
                          , const Location &where, CreateMode mode
                          , const std::string tilesetId)
{
    // TODO: check tilesetId for existence in the stack and consult with mode

    dbglog::thread_id(str(boost::format("%s->%s/%s")
                          % tileset.id()
                          % root.filename().string()
                          % tilesetId));


    LOG(info3) << "Adding tileset " << tileset.id() << " (from "
               << tileset.root() << ").";

    Tx tx;
    const auto workPath(root / TempDir / tilesetId);

    tx.add(workPath, root / TileSetDir / tilesetId);

    // create tileset at work path (overwrite any existing stuff here)
    auto dst(cloneTileSet(workPath, tileset, CreateMode::overwrite
                          , tilesetId));

    (void) dst;

    // TODO: create glues

    (void) mode;
    (void) where;
}

} } // namespace vadstena::vts
