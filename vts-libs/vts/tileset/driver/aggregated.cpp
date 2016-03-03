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
#include "../detail.hpp"
#include "./aggregated.hpp"

namespace vadstena { namespace vts { namespace driver {

namespace fs = boost::filesystem;

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

void unite(registry::IdSet &out, const registry::IdSet &in)
{
    out.insert(in.begin(), in.end());
}

} // namespace

AggregatedDriver::AggregatedDriver(const boost::filesystem::path &root
                                   , const AggregatedOptions &options
                                   , CreateMode mode
                                   , const TilesetId &tilesetId)
    : Driver(root, options, mode)
    , storage_(this->options().storagePath, OpenMode::readOnly)
{
    // compose tileset configuration
    const auto &tilesets(this->options().tilesets);

    TileSet::Properties properties;
    properties.id = tilesetId;
    properties.driverOptions = options;

    // try to get previous revision (and reuse)
    if (auto oldR = oldRevision()) {
        properties.revision = *oldR + 1;
    }

    // get reference frame
    properties.referenceFrame = storage_.getProperties().referenceFrame;
    properties.tileRange = TileRange(math::InvalidExtents{});

    // compose tile index and other properties
    TileIndex ti;
    bool first(true);
    for (const auto &tilesetId : tilesets) {
        LOG(info4) << "Adding tileset <" << tilesetId << ">.";

        auto ts(storage_.open(tilesetId));
        ti = unite(ti, ts.tileIndex());
        const auto &detail(ts.detail());
        const auto &tsProp(detail.properties);

        // TODO: unite all glues' tile indices as well

        // unite referenced registry entities
        unite(properties.credits, tsProp.credits);
        unite(properties.boundLayers, tsProp.boundLayers);

        // join various service data
        properties.lodRange = unite(properties.lodRange, tsProp.lodRange);
        properties.tileRange = unite(properties.tileRange, tsProp.tileRange);

        // TODO: spatial division extents

        // copy position from fist tileset
        if (first) {
            properties.position = tsProp.position;
            first = false;
        }
    }

    // save properties
    tileset::saveConfig(this->root() / filePath(File::config), properties);
    TileSet::Detail::saveTileIndex(this->root() / filePath(File::tileIndex)
                                   , ti, {});
}

AggregatedDriver::AggregatedDriver(const boost::filesystem::path &root
                                   , const AggregatedOptions &options)
    : Driver(root, options)
    , storage_(this->options().storagePath, OpenMode::readOnly)
{
}

AggregatedDriver::~AggregatedDriver() {}

OStream::pointer AggregatedDriver::output_impl(File)
{
    LOGTHROW(err2, storage::ReadOnlyError)
        << "This driver supports read access only.";
    return {};
}

IStream::pointer AggregatedDriver::input_impl(File type) const
{
    auto path(root() / filePath(type));
    LOG(info1) << "Loading from " << path << ".";
    return fileIStream(type, path);
}

OStream::pointer AggregatedDriver::output_impl(const TileId&, TileFile)
{
    LOGTHROW(err2, storage::ReadOnlyError)
        << "This driver supports read access only.";
    return {};
}

IStream::pointer AggregatedDriver::input_impl(const TileId &tileId
                                              , TileFile type)
    const
{
    (void) tileId;
    (void) type;
    return {};
}

FileStat AggregatedDriver::stat_impl(File type) const
{
    (void) type;
    return {};
}

FileStat AggregatedDriver::stat_impl(const TileId &tileId, TileFile type) const
{
    (void) tileId;
    (void) type;
    return {};
}

storage::Resources AggregatedDriver::resources_impl() const
{
    return {};
}


void AggregatedDriver::flush_impl() {
    LOGTHROW(err2, storage::ReadOnlyError)
        << "This driver supports read access only.";
}

void AggregatedDriver::drop_impl()
{
    LOGTHROW(err2, storage::ReadOnlyError)
        << "This driver supports read access only.";
}

} } } // namespace vadstena::vts::driver
