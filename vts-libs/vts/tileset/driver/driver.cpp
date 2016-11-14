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

// drivers:
#include "./plain.hpp"
#include "./aggregated-old.hpp"
#include "./remote.hpp"
#include "./local.hpp"

namespace vadstena { namespace vts {

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
               , const boost::any &options, CreateMode mode)
    : root_(absolute(root))
    , readOnly_(false)
    , configPath_(root_ / filePath(File::config))
    , extraConfigPath_(root_ / filePath(File::extraConfig))
    , registryPath_(root_ / filePath(File::registry))
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
        oldRevision_ = tileset::loadRevision(root_ / filePath(File::config));
    }
}

Driver::Driver(const boost::any &options, CreateMode)
    : root_()
    , readOnly_(true)
    , configPath_(root_ / filePath(File::config))
    , extraConfigPath_(root_ / filePath(File::extraConfig))
    , registryPath_(root_ / filePath(File::registry))
    , options_(options)
    , runnable_(), lastModified_()
{}

Driver::Driver(const boost::filesystem::path &root
               , const boost::any &options)
    : root_(absolute(root))
    , readOnly_(true)
    , configPath_(root_ / filePath(File::config))
    , extraConfigPath_(root_ / filePath(File::extraConfig))
    , registryPath_(root_ / filePath(File::registry))
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

Driver::~Driver()
{
}

bool Driver::externallyChanged() const
{
    return (rootStat_.changed(FileStat::stat(root_))
            || configStat_.changed(FileStat::stat(configPath_))
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
    } else if (auto o = boost::any_cast<const driver::OldAggregatedOptions>
               (&genericOptions))
    {
        return std::make_shared<driver::OldAggregatedDriver>
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
    if (auto o = boost::any_cast<const driver::OldAggregatedOptions>
        (&genericOptions))
    {
        return std::make_shared<driver::OldAggregatedDriver>
            (*o, cloneOptions);
    }

    LOGTHROW(err2, storage::BadFileFormat)
        << "Cannot create in-memory tileset: Invalid type of driver options: <"
        << genericOptions.type().name() << ">.";
    throw;
}

Driver::pointer Driver::open(const boost::filesystem::path &root)
{
    auto genericOptions(tileset::loadConfig(root / filePath(File::config))
                        .driverOptions);

    if (auto o = boost::any_cast<const driver::PlainOptions>
        (&genericOptions))
    {
        return std::make_shared<driver::PlainDriver>(root, *o);
    } else if (auto o = boost::any_cast<const driver::OldAggregatedOptions>
               (&genericOptions))
    {
        return std::make_shared<driver::OldAggregatedDriver>(root, *o);
    } else if (auto o = boost::any_cast<const driver::RemoteOptions>
               (&genericOptions))
    {
        return std::make_shared<driver::RemoteDriver>(root, *o);
    } else if (auto o = boost::any_cast<const driver::LocalOptions>
               (&genericOptions))
    {
        return std::make_shared<driver::LocalDriver>(root, *o);
    }

    LOGTHROW(err2, storage::BadFileFormat)
        << "Cannot open tileset at " << root
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
    } else if (auto o = boost::any_cast<const driver::OldAggregatedOptions>
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
    auto tmpPath(utility::addExtension(configPath, ".tmp"));
    tileset::saveConfig(tmpPath, config);
    fs::rename(tmpPath, configPath);
}

bool Driver::check(const boost::filesystem::path &root)
{
    try {
        tileset::loadConfig(root / filePath(File::config));
    } catch (storage::Error) {
        return false;
    }
    return true;
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

} } // namespace vadstena::vts
