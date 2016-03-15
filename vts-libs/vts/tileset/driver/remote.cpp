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
#include <boost/range/adaptor/reversed.hpp>

#include "dbglog/dbglog.hpp"

#include "utility/streams.hpp"
#include "utility/path.hpp"

#include "../../../storage/error.hpp"
#include "../../../storage/fstreams.hpp"
#include "../../../storage/io.hpp"
#include "../../io.hpp"
#include "../config.hpp"
#include "../detail.hpp"
#include "./remote.hpp"

namespace vadstena { namespace vts { namespace driver {

namespace fs = boost::filesystem;

namespace vs = vadstena::storage;

namespace {

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

} // namespace

RemoteDriverBase::RemoteDriverBase(const CloneOptions &cloneOptions)
{
    if (cloneOptions.lodRange()) {
        LOGTHROW(err2, storage::Error)
            << "REMOTE tileset driver doesn't support LOD sub ranging.";
    }
}

RemoteDriver::RemoteDriver(const boost::filesystem::path &root
                                   , const RemoteOptions &options
                                   , const CloneOptions &cloneOptions)
    : RemoteDriverBase(cloneOptions)
    , Driver(root, options, cloneOptions.mode())
    , fetcher_(this->options().url, {})
    , revision_()
{
    {
        auto properties(tileset::loadConfig(fetcher_.input(File::config)));
        if (cloneOptions.tilesetId()) {
            properties.id = *cloneOptions.tilesetId();
        }
        properties.driverOptions = options;
        tileset::saveConfig(this->root() / filePath(File::config)
                            , properties);
        revision_ = properties.revision;
    }

    // clone tile index
    copyFile(fetcher_.input(File::tileIndex), output(File::tileIndex));

    // and load it
    tileset::loadTileSetIndex(tsi_, *this);

    // make me read-only
    readOnly(true);
}

RemoteDriver::RemoteDriver(const boost::filesystem::path &root
                       , const RemoteOptions &options)
    : Driver(root, options)
    , fetcher_(this->options().url, {})
    , revision_()
{
    {
        auto properties(tileset::loadConfig(*this));
        revision_ = properties.revision;
    }
    tileset::loadTileSetIndex(tsi_, *this);
}

RemoteDriver::RemoteDriver(const boost::filesystem::path &root
                                   , const RemoteOptions &options
                                   , const CloneOptions &cloneOptions
                                   , const RemoteDriver &src)
    : RemoteDriverBase(cloneOptions)
    , Driver(root, options, cloneOptions.mode())
    , fetcher_(this->options().url, {})
{
    // update and save properties
    {
        auto properties(tileset::loadConfig(src));
        if (cloneOptions.tilesetId()) {
            properties.id = *cloneOptions.tilesetId();
        }
        properties.driverOptions = options;
        tileset::saveConfig(this->root() / filePath(File::config)
                            , properties);
        revision_ = properties.revision;
    }

    // clone tile index
    copyFile(src.input(File::tileIndex), output(File::tileIndex));

    // and load it
    tileset::loadTileSetIndex(tsi_, *this);

    // make me read-only
    readOnly(true);
}

Driver::pointer
RemoteDriver::clone_impl(const boost::filesystem::path &root
                             , const CloneOptions &cloneOptions)
    const
{
    return std::make_shared<RemoteDriver>
        (root, options(), cloneOptions, *this);
}

RemoteDriver::~RemoteDriver() {}

OStream::pointer RemoteDriver::output_impl(File type)
{
    if (readOnly()) {
        LOGTHROW(err2, storage::ReadOnlyError)
            << "This driver supports read access only.";
    }

    const auto path(root() / filePath(type));
    LOG(info1) << "Saving to " << path << ".";
    return fileOStream(type, path);
}

IStream::pointer RemoteDriver::input_impl(File type) const
{
    auto path(root() / filePath(type));
    LOG(info1) << "Loading from " << path << ".";
    return fileIStream(type, path);
}

OStream::pointer RemoteDriver::output_impl(const TileId&, TileFile)
{
    LOGTHROW(err2, storage::ReadOnlyError)
        << "This driver supports read access only.";
    return {};
}

IStream::pointer RemoteDriver::input_impl(const TileId &tileId
                                        , TileFile type)
    const
{
    return fetcher_.input(tileId, type, revision_);
}

FileStat RemoteDriver::stat_impl(File type) const
{
    const auto name(filePath(type));
    const auto path(root() / name);
    LOG(info1) << "Statting " << path << ".";
    return FileStat::stat(path);
}

FileStat RemoteDriver::stat_impl(const TileId &tileId, TileFile type) const
{
    (void) tileId;
    (void) type;
    return {};
}

storage::Resources RemoteDriver::resources_impl() const
{
    // nothing
    return {};
}

void RemoteDriver::flush_impl() {
    LOGTHROW(err2, storage::ReadOnlyError)
        << "This driver supports read access only.";
}

void RemoteDriver::drop_impl()
{
    LOGTHROW(err2, storage::ReadOnlyError)
        << "This driver supports read access only.";
}

std::string RemoteDriver::info_impl() const
{
    auto o(options());
    std::ostringstream os;
    os << "remote (URL=" << o.url << ")";
    return os.str();
}

} } } // namespace vadstena::vts::driver
