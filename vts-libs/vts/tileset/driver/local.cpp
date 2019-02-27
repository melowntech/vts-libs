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

#include "dbglog/dbglog.hpp"

#include "utility/streams.hpp"
#include "utility/path.hpp"

#include "../../../storage/error.hpp"
#include "../../../storage/fstreams.hpp"
#include "../../../storage/io.hpp"
#include "../../io.hpp"
#include "../config.hpp"
#include "../detail.hpp"
#include "local.hpp"

namespace vtslibs { namespace vts { namespace driver {

namespace fs = boost::filesystem;

namespace vs = vtslibs::storage;

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
    , Driver(root, cloneOptions.openOptions(), options, cloneOptions.mode())
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
                         , const OpenOptions &openOptions
                         , const LocalOptions &options)
    : Driver(root, openOptions, options)
    , driver_(Driver::open(options.path, openOptions))
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

IStream::pointer LocalDriver::input_impl(File type, const NullWhenNotFound_t&)
    const
{
    switch (type) {
    case File::config:
    case File::extraConfig: {
        auto path(root() / filePath(type));
        LOG(info1) << "Loading from " << path << ".";
        return fileIStream(type, path, NullWhenNotFound);
    }

    default: break;
    }
    return driver_->input(type, NullWhenNotFound);
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

IStream::pointer LocalDriver::input_impl(const TileId &tileId, TileFile type
                                         , const NullWhenNotFound_t&)
    const
{
    return driver_->input(tileId, type, NullWhenNotFound);
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

bool LocalDriver::reencode(const boost::filesystem::path &root
                           , const LocalOptions &driverOptions
                           , const ReencodeOptions &options
                           , const std::string &prefix)
{
    LOG(info3)
        << prefix << "Following " << root << " -> "
        << driverOptions.path << ".";
    Driver::reencode(driverOptions.path, options, prefix);
    return !options.dryRun && !options.cleanup;
}

LocalDriver::LocalDriver(PrivateTag, const boost::filesystem::path &root
                         , const OpenOptions &openOptions
                         , const LocalOptions &options
                         , Driver::pointer owned)
    : Driver(root, openOptions, options)
    , driver_(std::move(owned))
{}

void LocalDriver::open(const boost::filesystem::path &root
                       , const OpenOptions &openOptions
                       , const LocalOptions &options
                       , const DriverOpenCallback::pointer &callback)
{
    struct DriverOpener : DriverOpenCallback {
        DriverOpener(const boost::filesystem::path &root
                     , const OpenOptions &openOptions
                     , const LocalOptions &options
                     , const DriverOpenCallback::pointer &callback)
            : root(root), openOptions(openOptions), options(options)
            , callback(callback)
        {}

        virtual void done(Driver::pointer driver) {
            callback->done(std::make_shared<LocalDriver>
                           (LocalDriver::PrivateTag(), root, openOptions
                            , options, std::move(driver)));
        }

        virtual void error(const std::exception_ptr &exc) {
            // forward
            callback->error(exc);
        }

        virtual void openStorage(const boost::filesystem::path &path
                                 , const StorageOpenCallback::pointer
                                 &callback)
        {
            // forward
            return this->callback->openStorage(path, callback);
        }

        virtual void openDriver(const boost::filesystem::path &path
                                , const OpenOptions &openOptions
                                , const DriverOpenCallback::pointer
                                &callback)
        {
            // forward
            return this->callback->openDriver(path, openOptions, callback);
        }

        const boost::filesystem::path root;
        const OpenOptions openOptions;
        const LocalOptions options;
        const DriverOpenCallback::pointer callback;
    };

    callback->openDriver
        (options.path, openOptions
         , std::make_shared<DriverOpener>
         (root, openOptions, options, callback));
}

} } } // namespace vtslibs::vts::driver
