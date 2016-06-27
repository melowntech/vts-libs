#include <stdexcept>
#include <limits>
#include <type_traits>
#include <fstream>

#include "dbglog/dbglog.hpp"

#include "utility/streams.hpp"
#include "utility/path.hpp"

#include "../../../storage/error.hpp"
#include "../../../storage/fstreams.hpp"
#include "../../../storage/io.hpp"
#include "../../io.hpp"
#include "../config.hpp"
#include "../detail.hpp"
#include "./local.hpp"

namespace vadstena { namespace vts { namespace driver {

namespace fs = boost::filesystem;

namespace vs = vadstena::storage;

namespace {

const std::string ConfigName("tileset.conf");
const std::string ExtraConfigName("extra.conf");

const std::string filePath(File type)
{
    switch (type) {
    case File::config: return ConfigName;
    case File::extraConfig: return ExtraConfigName;
    default: break;
    }
    throw "unknown file type";
}

} // namespace

LocalDriverBase::LocalDriverBase(const CloneOptions &cloneOptions)
{
    if (cloneOptions.lodRange()) {
        LOGTHROW(err2, storage::Error)
            << "LOCAL tileset driver doesn't support LOD sub ranging.";
    }
}

LocalDriver::LocalDriver(const boost::filesystem::path &root
                         , const LocalOptions &options
                         , const CloneOptions &cloneOptions)
    : LocalDriverBase(cloneOptions)
    , Driver(root, options, cloneOptions.mode())
    , driver_(Driver::open(options.path))
{
    {
        auto properties(tileset::loadConfig((driver_->input(File::config))));
        if (cloneOptions.tilesetId()) {
            properties.id = *cloneOptions.tilesetId();
        }
        auto opts(options);
        opts.path = fs::absolute(opts.path);
        properties.driverOptions = opts;
        tileset::saveConfig(this->root() / filePath(File::config)
                            , properties);
    }

    // make me read-only
    readOnly(true);
}

LocalDriver::LocalDriver(const boost::filesystem::path &root
                       , const LocalOptions &options)
    : Driver(root, options)
    , driver_(Driver::open(options.path))
{}

Driver::pointer
LocalDriver::clone_impl(const boost::filesystem::path &root
                        , const CloneOptions &cloneOptions)
    const
{
    // create new tileset with the same configuration
    auto opts(this->options());
    opts.path = fs::absolute(opts.path);

    return std::make_shared<LocalDriver>(root, opts, cloneOptions);
}

LocalDriver::~LocalDriver() {}

OStream::pointer LocalDriver::output_impl(File)
{
    LOGTHROW(err2, storage::ReadOnlyError)
        << "This driver supports read access only.";
    return {};
}

IStream::pointer LocalDriver::input_impl(File type) const
{
    switch (type) {
    case File::config:
    case File::extraConfig: {
        auto path(root() / filePath(type));
        LOG(info1) << "Loading from " << path << ".";
        return fileIStream(type, path);
    }

    default: break;
    }
    return driver_->input(type);
}

OStream::pointer LocalDriver::output_impl(const TileId&, TileFile)
{
    LOGTHROW(err2, storage::ReadOnlyError)
        << "This driver supports read access only.";
    return {};
}

IStream::pointer LocalDriver::input_impl(const TileId &tileId
                                        , TileFile type)
    const
{
    return driver_->input(tileId, type);
}

FileStat LocalDriver::stat_impl(File type) const
{
    switch (type) {
    case File::config:
    case File::extraConfig: {
        const auto name(filePath(type));
        const auto path(root() / name);
        LOG(info1) << "Statting " << path << ".";
        return FileStat::stat(path);
    }

    default: break;
    }
    return driver_->stat(type);
}

FileStat LocalDriver::stat_impl(const TileId &tileId, TileFile type) const
{
    return driver_->stat(tileId, type);
}

storage::Resources LocalDriver::resources_impl() const
{
    // nothing
    return driver_->resources();
}

void LocalDriver::flush_impl() {
    LOGTHROW(err2, storage::ReadOnlyError)
        << "This driver supports read access only.";
}

void LocalDriver::drop_impl()
{
    LOGTHROW(err2, storage::ReadOnlyError)
        << "This driver supports read access only.";
}

std::string LocalDriver::info_impl() const
{
    auto o(options());
    std::ostringstream os;
    os << "local (path=" << o.path << ")";
    return os.str();
}

boost::any LocalOptions::relocate(const RelocateOptions &options
                                  , const std::string &prefix) const
{
    auto res(options.apply(path.string()));

    auto ret([&]() -> boost::any
    {
        if (!res.replacement) {
            LOG(info3)
                << prefix << "Nothing to do with path " << path << ".";
            return {};
        }

        // update
        if (options.dryRun) {
            LOG(info3)
                << prefix << "Would relocate local path " << path
                << " to path "
                << *res.replacement << ".";
            return {};
        }

        auto out(*this);
        LOG(info3)
            << prefix << "Relocating local path " << path << " to path "
            << *res.replacement << ".";
        out.path = *res.replacement;
        return out;
    }());

    // descend
    Driver::relocate(res.follow, options, prefix + "    ");

    return ret;
}

} } } // namespace vadstena::vts::driver
