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
#include "../../../storage/fstreams.hpp"
#include "../../io.hpp"
#include "../config.hpp"
#include "../driver.hpp"
#include "../detail.hpp"

// drivers:
#include "./plain.hpp"
#include "./aggregated.hpp"

namespace vadstena { namespace vts {

namespace fs = boost::filesystem;

namespace {
    const std::uint8_t DefaultBinaryOrder(5);

    const std::string ConfigName("tileset.conf");
    const std::string ExtraConfigName("extra.conf");
    const std::string TileIndexName("tileset.index");

    const std::string filePath(File type)
    {
        switch (type) {
        case File::config: return ConfigName;
        case File::extraConfig: return ExtraConfigName;
        case File::tileIndex: return TileIndexName;
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

Driver::Driver(const boost::filesystem::path &root
               , const boost::any &options)
    : root_(absolute(root))
    , readOnly_(true)
    , configPath_(root_ / filePath(File::config))
    , extraConfigPath_(root_ / filePath(File::extraConfig))
    , options_(options)
    , rootStat_(FileStat::stat(root_))
    , configStat_(FileStat::stat(configPath_))
    , extraConfigStat_(FileStat::stat(extraConfigPath_, std::nothrow))
    , runnable_()
    , lastModified_(std::max({ rootStat_.lastModified, configStat_.lastModified
                    , extraConfigStat_.lastModified }))
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
                                        (extraConfigPath_, std::nothrow)));
}

bool Driver::readOnly() const
{
    return readOnly_;
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
                               , CreateMode mode)
{
    if (auto o = boost::any_cast<const driver::PlainDriverOptions>
        (&genericOptions))
    {
        return std::make_shared<driver::PlainDriver>(root, *o, mode);
    } else if (auto o = boost::any_cast<const driver::AggregatedDriverOptions>
               (&genericOptions))
    {
        return std::make_shared<driver::AggregatedDriver> (root, *o, mode);
    }

    LOGTHROW(err2, storage::BadFileFormat)
        << "Cannot create tileset at " << root
        << ": Invalid type of driver options: <"
        << genericOptions.type().name() << ">.";
    throw;
}

Driver::pointer Driver::open(const boost::filesystem::path &root)
{
    auto genericOptions(tileset::loadConfig(root / filePath(File::config))
                        .driverOptions);

    if (auto o = boost::any_cast<const driver::PlainDriverOptions>
        (&genericOptions))
    {
        return std::make_shared<driver::PlainDriver>(root, *o);
    } else if (auto o = boost::any_cast<const driver::AggregatedDriverOptions>
               (&genericOptions))
    {
        return std::make_shared<driver::AggregatedDriver> (root, *o);
    }

    LOGTHROW(err2, storage::BadFileFormat)
        << "Cannot open tileset at " << root
        << ": Invalid type of driver options: <"
        << genericOptions.type().name() << ">.";
    throw;
}

} } // namespace vadstena::vts
