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

namespace vadstena { namespace vts {

namespace fs = boost::filesystem;

namespace {
    const std::uint8_t DefaultBinaryOrder(5);

    const std::string ConfigName("config.json");
    const std::string TileIndexName("index.bin");

    const std::string filePath(File type)
    {
        switch (type) {
        case File::config: return ConfigName;
        case File::tileIndex: return TileIndexName;
        }
        throw "unknown file type";
    }

    boost::uuids::uuid generateUuid() {
        // generate random uuid
        return boost::uuids::random_generator()();
    }
} // namespace

namespace driver {

long Options::calculateMask(std::uint8_t order)
{
    long value(0);
    for (long bit(1l); order; --order, bit <<= 1) {
        value |= bit;
    }
    return value;
}

boost::uuids::uuid Options::generateUuid() {
    // generate random uuid
    return boost::uuids::random_generator()();
}

} // namespace driver

Driver::Driver(const boost::filesystem::path &root
               , CreateMode mode, const driver::Options &options)
    : root_(absolute(root))
    , configPath_(root_ / filePath(File::config))
    , options_(options.binaryOrder()) // we have to generate new UUID
    , cache_(root_, options_, false)
{
    if (!create_directories(root_)) {
        // directory already exists -> fail if mode says so
        if (mode == CreateMode::failIfExists) {
            LOGTHROW(err2, storage::TileSetAlreadyExists)
                << "Tile set at " << root_ << " already exists.";
        }
    }
}

Driver::Driver(const boost::filesystem::path &root)
    : root_(absolute(root))
    , configPath_(root_ / filePath(File::config))
    , options_(vts::loadConfig(configPath_).driverOptions)
    , cache_(root_, options_, true)
    , openStat_(FileStat::stat(configPath_))
{
}

Driver::~Driver()
{
}

OStream::pointer Driver::output(File type)
{
    checkRunning();

    const auto name(filePath(type));
    // no transaction -> plain file
    const auto path(root_ / name);
    LOG(info1) << "Saving to " << path << ".";
    return fileOStream(type, path);
}

IStream::pointer Driver::input(File type) const
{
    checkRunning();

    const auto name(filePath(type));

    auto path(root_ / name);
    LOG(info1) << "Loading from " << path << ".";
    return fileIStream(type, path);
}

OStream::pointer Driver::output(const TileId tileId, TileFile type)
{
    checkRunning();

    return cache_.output(tileId, type);
}

IStream::pointer Driver::input(const TileId tileId, TileFile type)
    const
{
    return cache_.input(tileId, type);
}

FileStat Driver::stat(File type) const
{
    checkRunning();

    const auto name(filePath(type));
    const auto path(root_ / name);
    LOG(info1) << "Statting " << path << ".";
    return FileStat::stat(path);
}

FileStat Driver::stat(const TileId tileId, TileFile type) const
{
    checkRunning();

    return cache_.stat(tileId, type);
}

storage::Resources Driver::resources() const
{
    return cache_.resources();
}


void Driver::flush()
{
    cache_.flush();
}

bool Driver::externallyChanged() const
{
    return openStat_.changed(FileStat::stat(configPath_));
}

void Driver::wannaWrite(const std::string &what) const
{
    if (cache_.readOnly()) {
        LOGTHROW(err2, storage::ReadOnlyError)
            << "Cannot " << what << ": storage is read-only.";
    }
}

void Driver::drop()
{
    // remove whole root directory
    remove_all(root_);
}

void Driver::notRunning() const
{
    LOGTHROW(warn2, storage::Interrupted)
        << "Operation has been interrupted.";
}

} } // namespace vadstena::vts
