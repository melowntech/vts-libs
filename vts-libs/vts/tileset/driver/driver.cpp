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
#include <stdexcept>
#include <limits>
#include <type_traits>
#include <fstream>

#include <boost/filesystem.hpp>
#include <boost/format.hpp>
#include <boost/utility/in_place_factory.hpp>
#include <boost/uuid/string_generator.hpp>
#include <boost/uuid/random_generator.hpp>
#include <boost/uuid/uuid_io.hpp>

#include "dbglog/dbglog.hpp"

#include "utility/streams.hpp"
#include "utility/path.hpp"

#include "../../../storage/error.hpp"
#include "../../io.hpp"
#include "../config.hpp"
#include "../driver.hpp"
#include "../detail.hpp"

#include "runcallback.hpp"

// drivers:
#include "plain.hpp"
#include "aggregated.hpp"
#include "remote.hpp"
#include "local.hpp"

namespace vtslibs { namespace vts {

namespace fs = boost::filesystem;

namespace {
    const std::uint8_t DefaultBinaryOrder(5);

    const std::string ConfigName("tileset.conf");
    const std::string ExtraConfigName("extra.conf");
    const std::string TileIndexName("tileset.index");
    const std::string RegistryName("tileset.registry");

    const std::string filePath(File type)
    {
        switch (type) {
        case File::config: return ConfigName;
        case File::extraConfig: return ExtraConfigName;
        case File::tileIndex: return TileIndexName;
        case File::registry: return RegistryName;
        default: break;
        }
        throw "unknown file type";
    }

    boost::uuids::uuid generateUuid() {
        // generate random uuid
        return boost::uuids::random_generator()();
    }
} // namespace

Driver::Driver(const boost::filesystem::path &root
               , const OpenOptions &openOptions
               , const boost::any &options, CreateMode mode)
    : root_(absolute(root))
    , readOnly_(false)
    , configPath_(root_ / filePath(File::config))
    , extraConfigPath_(root_ / filePath(File::extraConfig))
    , registryPath_(root_ / filePath(File::registry))
    , openOptions_(openOptions)
    , options_(options)
    , runnable_(), lastModified_()
{
    if (!create_directories(root_)) {
        // directory already exists -> fail if mode says so
        if (mode == CreateMode::failIfExists) {
            LOGTHROW(err2, storage::TileSetAlreadyExists)
                << "Tile set at " << root_ << " already exists.";
        }

        // OK, we can overwrite; cache contents of old config (if any)
        oldRevision_ = oldRevision(root_);
    }
}

Driver::Driver(const OpenOptions &openOptions
               , const boost::any &options
               , CreateMode)
    : root_()
    , readOnly_(true)
    , configPath_(root_ / filePath(File::config))
    , extraConfigPath_(root_ / filePath(File::extraConfig))
    , registryPath_(root_ / filePath(File::registry))
    , openOptions_(openOptions)
    , options_(options)
    , runnable_(), lastModified_()
{}

Driver::Driver(const boost::filesystem::path &root
               , const OpenOptions &openOptions
               , const boost::any &options)
    : root_(absolute(root))
    , readOnly_(true)
    , configPath_(root_ / filePath(File::config))
    , extraConfigPath_(root_ / filePath(File::extraConfig))
    , registryPath_(root_ / filePath(File::registry))
    , openOptions_(openOptions)
    , options_(options)
    , rootStat_(FileStat::stat(root_))
    , configStat_(FileStat::stat(configPath_))
    , extraConfigStat_(FileStat::stat(extraConfigPath_, std::nothrow))
    , registryStat_(FileStat::stat(registryPath_, std::nothrow))
    , runnable_()
    , lastModified_(std::max({ rootStat_.lastModified, configStat_.lastModified
                    , extraConfigStat_.lastModified
                    , registryStat_.lastModified}))
{
}

Driver::~Driver() {}

bool Driver::externallyChanged() const
{
    return (rootStat_.changed(FileStat::stat(root_, std::nothrow))
            || configStat_.changed(FileStat::stat(configPath_, std::nothrow))
            || extraConfigStat_.changed(FileStat::stat
                                        (extraConfigPath_, std::nothrow))
            || registryStat_.changed(FileStat::stat
                                        (registryPath_, std::nothrow)));
}

bool Driver::readOnly() const
{
    return readOnly_;
}

void Driver::readOnly(bool value)
{
    readOnly_ = value;
}

void Driver::wannaWrite(const std::string &what) const
{
    if (readOnly()) {
        LOGTHROW(err2, storage::ReadOnlyError)
            << "Cannot " << what << ": storage is read-only.";
    }
}

void Driver::notRunning() const
{
    LOGTHROW(warn2, storage::Interrupted)
        << "Operation has been interrupted.";
}

Driver::pointer Driver::create(const boost::filesystem::path &root
                               , const boost::any &genericOptions
                               , const CloneOptions &cloneOptions)
{
    if (auto o = boost::any_cast<const driver::PlainOptions>
        (&genericOptions))
    {
        return std::make_shared<driver::PlainDriver>
            (root, *o, cloneOptions);
    } else if (auto o = boost::any_cast<const driver::AggregatedOptions>
               (&genericOptions))
    {
        return std::make_shared<driver::AggregatedDriver>
            (root, *o, cloneOptions);
    } else if (auto o = boost::any_cast<const driver::RemoteOptions>
               (&genericOptions))
    {
        return std::make_shared<driver::RemoteDriver>
            (root, *o, cloneOptions);
    } else if (auto o = boost::any_cast<const driver::LocalOptions>
               (&genericOptions))
    {
        return std::make_shared<driver::LocalDriver>
            (root, *o, cloneOptions);
    }

    LOGTHROW(err2, storage::BadFileFormat)
        << "Cannot create tileset at " << root
        << ": Invalid type of driver options: <"
        << genericOptions.type().name() << ">.";
    throw;
}

Driver::pointer Driver::create(const boost::any &genericOptions
                               , const CloneOptions &cloneOptions)
{
    if (auto o = boost::any_cast<const driver::AggregatedOptions>
        (&genericOptions))
    {
        return std::make_shared<driver::AggregatedDriver>
            (*o, cloneOptions);
    }

    LOGTHROW(err2, storage::BadFileFormat)
        << "Cannot create in-memory tileset: Invalid type of driver options: <"
        << genericOptions.type().name() << ">.";
    throw;
}

Driver::pointer Driver::open(const boost::filesystem::path &root
                             , const OpenOptions &openOptions)
{
    return open(root, tileset::loadConfig(root / filePath(File::config))
                .driverOptions, openOptions);
}

Driver::pointer Driver::open(const boost::filesystem::path &root
                             , const BareConfigTag&
                             , const OpenOptions &openOptions)
{
    return open(root, tileset::loadDriver(root / filePath(File::config))
                , openOptions);
}

/** Opens driver for existing dataset, with driver options available.
 */
Driver::pointer Driver::open(const boost::filesystem::path &root
                             , const boost::any &genericOptions
                             , const OpenOptions &openOptions)
{
    if (auto o = boost::any_cast<const driver::PlainOptions>
        (&genericOptions))
    {
        return std::make_shared<driver::PlainDriver>(root, openOptions, *o);
    } else if (auto o = boost::any_cast<const driver::AggregatedOptions>
               (&genericOptions))
    {
        return std::make_shared<driver::AggregatedDriver>
            (root, openOptions, *o);
    } else if (auto o = boost::any_cast<const driver::RemoteOptions>
               (&genericOptions))
    {
        return std::make_shared<driver::RemoteDriver>
            (root, openOptions, *o);
    } else if (auto o = boost::any_cast<const driver::LocalOptions>
               (&genericOptions))
    {
        return std::make_shared<driver::LocalDriver>
            (root, openOptions, *o);
    }

    LOGTHROW(err2, storage::BadFileFormat)
        << "Cannot open tileset at " << root
        << ": Invalid type of driver options: <"
        << genericOptions.type().name() << ">.";
    throw;
}

Driver::pointer Driver::configReader(const boost::filesystem::path &root
                                     , const OpenOptions &openOptions)
{
    return configReader(root
                        , tileset::loadConfig(root / filePath(File::config))
                        .driverOptions, openOptions);;
}

Driver::pointer Driver::configReader(const boost::filesystem::path &root
                                     , const boost::any &genericOptions
                                     , const OpenOptions &openOptions)
{
    if (auto o = boost::any_cast<const driver::PlainOptions>
        (&genericOptions))
    {
        return std::make_shared<driver::PlainDriver>(root, openOptions, *o);
    } else if (boost::any_cast<const driver::AggregatedOptions>
               (&genericOptions))
    {
        // fake driver via plain driver
        return std::make_shared<driver::PlainDriver>
            (root, openOptions, driver::PlainOptions(0));
    } else if (boost::any_cast<const driver::RemoteOptions>(&genericOptions)) {
        // fake driver via plain driver
        return std::make_shared<driver::PlainDriver>
            (root, openOptions, driver::PlainOptions(0));
    } else if (auto o = boost::any_cast<const driver::LocalOptions>
               (&genericOptions))
    {
        return std::make_shared<driver::LocalDriver>
            (root, openOptions, *o);
    }

    LOGTHROW(err2, storage::BadFileFormat)
        << "Cannot open tileset config reader at " << root
        << ": Invalid type of driver options: <"
        << genericOptions.type().name() << ">.";
    throw;
}

namespace {

boost::any relocateOptions(const boost::any &options
                           , const RelocateOptions &relocateOptions
                           , const std::string &prefix)
{
    if (auto o = boost::any_cast<const driver::PlainOptions>(&options))
    {
        return o->relocate(relocateOptions, prefix);
    } else if (auto o = boost::any_cast<const driver::AggregatedOptions>
               (&options))
    {
        return o->relocate(relocateOptions, prefix);
    } else if (auto o = boost::any_cast<const driver::RemoteOptions>
               (&options))
    {
        return o->relocate(relocateOptions, prefix);
    } else if (auto o = boost::any_cast<const driver::LocalOptions>
               (&options))
    {
        return o->relocate(relocateOptions, prefix);
    }

    LOGTHROW(err2, storage::BadFileFormat)
        << prefix
        << "Cannot relocate tileset: Invalid type of driver options: <"
        << options.type().name() << ">.";
    throw;
}

void saveConfig(const fs::path &configPath
                , const FullTileSetProperties &config)
{
    // safe save
    auto tmpPath(utility::addExtension(configPath, ".tmp"));
    tileset::saveConfig(tmpPath, config);
    fs::rename(tmpPath, configPath);
}

} // namespace

void Driver::relocate(const boost::filesystem::path &root
                      , const RelocateOptions &ro
                      , const std::string &prefix)
{
    if (ro.dryRun) {
        LOG(info3) << prefix << "Simulating relocation of tileset "
                   << root << ".";
    } else {
        LOG(info3) << prefix << "Relocating tileset " << root << ".";
    }

    const auto configPath(root / filePath(File::config));

    auto config(tileset::loadConfig(configPath));

    auto relocated(relocateOptions(config.driverOptions, ro
                                   , prefix + "    "));

    if (relocated.empty()) { return; }

    // update config
    config.driverOptions = relocated;

    // safe save
    saveConfig(configPath, config);
}

namespace {

bool checkReencodeMarker(const boost::filesystem::path &root
                         , const ReencodeOptions &ro
                         , const std::string &prefix)
{
    (void) prefix;
    const auto marker(root / (ro.tag + ".marker"));
    bool exists(fs::exists(marker));
    if (exists && !ro.cleanup) {
        LOG(info3) << prefix << "    Tileset " << root
                   << " already reencoded.";
        return false;
    }

    if (!exists && ro.cleanup) {
        LOG(info3) << prefix << "    Tileset " << root
                   << " not reencoded, cannot clean up.";
        return false;
    }

    return true;
}

void setReencodeMarker(const boost::filesystem::path &root
                       , const ReencodeOptions &ro
                       , const std::string &prefix)
{
    (void) prefix;
    const auto marker(root / (ro.tag + ".marker"));
    if (ro.dryRun) { return; }

    if (ro.cleanup) {
        // cleanup -> remove marker
        boost::system::error_code ec;
        fs::remove(marker, ec);
    } else {
        // store marker
        std::ofstream of(marker.string());
        of.flush();
    }
}

} //namespace

void Driver::reencode(const boost::filesystem::path &root
                      , const ReencodeOptions &ro
                      , const std::string &prefix)
{
    if (ro.cleanup) {
        if (ro.dryRun) {
            LOG(info3) << prefix << "Simulating reencode cleanup of tileset "
                       << root << ".";
        } else {
            LOG(info3)
                << prefix << "Cleaning up reencode of tileset " << root << ".";
        }
    } else {
        if (ro.dryRun) {
            LOG(info3) << prefix << "Simulating reencode cleanup of tileset "
                       << root << ".";
        } else {
            LOG(info3) << prefix << "Reencoding tileset " << root << ".";
        }
    }

    const auto configPath(root / filePath(File::config));

    auto config(tileset::loadConfig(configPath));
    const auto &options(config.driverOptions);

    if (!checkReencodeMarker(root, ro, prefix)) { return; }

    bool bumpRevision(false);

    // open tileset
    if (auto o = boost::any_cast<const driver::PlainOptions>(&options))
    {
        bumpRevision
            = driver::PlainDriver::reencode(root, *o, ro, prefix);
    } else if (auto o = boost::any_cast<const driver::AggregatedOptions>
               (&options))
    {
        bumpRevision
            = driver::AggregatedDriver::reencode(root, *o, ro, prefix);
    } else if (auto o = boost::any_cast<const driver::RemoteOptions>
               (&options))
    {
        bumpRevision
            = driver::RemoteDriver::reencode(root, *o, ro, prefix);
    } else if (auto o = boost::any_cast<const driver::LocalOptions>
               (&options))
    {
        bumpRevision
            = driver::LocalDriver::reencode(root, *o, ro, prefix);
    }

    if (bumpRevision) {
        // bump revision
        ++config.revision;
        saveConfig(configPath, config);
    }

    setReencodeMarker(root, ro, prefix);
}

bool Driver::check(const boost::filesystem::path &root)
{
    // try config existence
    try {
        return fs::exists(root / filePath(File::config));
    } catch (const fs::filesystem_error&) {}
    return false;
}

bool Driver::check(const boost::filesystem::path &root
                   , const std::string &mime)
{
    if (mime == "inode/directory") { return check(root); }
    return false;
}

IStream::pointer Driver::input_impl(const std::string &name) const
{
    LOGTHROW(err1, storage::NoSuchFile)
        << "This driver doesn't serve file \"" << name << "\".";
    throw;
}

IStream::pointer Driver::input_impl(const std::string &name
                                    , const NullWhenNotFound_t&)
    const
{
    LOG(err1) << "This driver doesn't serve file \"" << name << "\".";
    return {};
}

FileStat Driver::stat_impl(const std::string &name) const
{
    LOGTHROW(err1, storage::NoSuchFile)
        << "This driver doesn't serve file \"" << name << "\".";
    throw;
}

namespace driver {

MapConfigOverride::MapConfigOverride(const boost::any &options)
{
    if (auto o = boost::any_cast<const driver::RemoteOptions>
        (&options))
    {
        // force remote URL as a template prefix and since URL is not a local
        // path mark it as unchangeable
        root = o->url;
    }
}

} // namespace driver

void Driver::open(const boost::filesystem::path &root
                  , const boost::any &genericOptions
                  , const OpenOptions &openOptions
                  , const DriverOpenCallback::pointer &callback)
{
    // distribute
    if (auto o = boost::any_cast<const driver::PlainOptions>
        (&genericOptions))
    {
        callback->done
            (std::make_shared<driver::PlainDriver>
             (root, openOptions, *o));
        return;

    } else if (auto o = boost::any_cast<const driver::AggregatedOptions>
               (&genericOptions))
    {
        return driver::AggregatedDriver::open(root, openOptions, *o, callback);

    } else if (auto o = boost::any_cast<const driver::RemoteOptions>
               (&genericOptions))
    {
        callback->done
            (std::make_shared<driver::RemoteDriver>
             (root, openOptions, *o));
        return;

    } else if (auto o = boost::any_cast<const driver::LocalOptions>
               (&genericOptions))
    {
        return driver::LocalDriver::open(root, openOptions, *o, callback);
    }

    // invalid type
    LOGTHROW(err2, storage::BadFileFormat)
        << "Cannot open tileset at " << root
        << ": Invalid type of driver options: <"
        << genericOptions.type().name() << ">.";
}

// async open declared in vts.hpp

void openTilesetDriver(const boost::filesystem::path &root
                       , const OpenOptions &openOptions
                       , const DriverOpenCallback::pointer &callback)
{
    try {
        // open driver options
        const auto options
            (tileset::loadConfig(root / filePath(File::config)).driverOptions);
        Driver::open(root, options, openOptions, callback);
    } catch (...) {
        // report error
        callback->error(std::current_exception());
    }
}

boost::optional<unsigned int>
Driver::oldRevision(const boost::filesystem::path &root)
{
    return tileset::loadRevision(root / filePath(File::config));
}

void Driver::input_impl(const TileId &tileId, TileFile type
                        , const InputCallback &cb
                        , const IStream::pointer *notFound) const
{
    if (!notFound) {
        // regular call
        return runCallback([&]() { return input_impl(tileId, type); }, cb);
    }

    // not-found signalling version
    return runCallback([&]() -> IStream::pointer
    {
        auto is(input_impl(tileId, type, NullWhenNotFound));
        return (is ? is : *notFound);
    }, cb);
}

void Driver::stat_impl(const TileId &tileId, TileFile type
                       , const StatCallback &cb) const
{
    runCallback([&]() { return stat_impl(tileId, type); }, cb);
}

} } // namespace vtslibs::vts
