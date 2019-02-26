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
#include "remote.hpp"
#include "runcallback.hpp"

namespace vtslibs { namespace vts { namespace driver {

namespace fs = boost::filesystem;

namespace vs = vtslibs::storage;

namespace {

const std::string ConfigName("tileset.conf");
const std::string ExtraConfigName("extra.conf");
const std::string TileIndexName("tileset.index");
const std::string RegistryName("tileset.registry");

const std::string filePath(File type) {
    switch (type) {
    case File::config: return ConfigName;
    case File::extraConfig: return ExtraConfigName;
    case File::tileIndex: return TileIndexName;
    case File::registry: return RegistryName;
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
    , Driver(root, cloneOptions.openOptions(), options, cloneOptions.mode())
    , fetcher_(this->options().url, cloneOptions.openOptions())
    , revision_()
{
    // asynchronous in nature
    capabilities().async = true;

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

    // clone registry if exists
    if (auto registry = fetcher_.input(File::registry, false)) {
        copyFile(registry, output(File::registry));
    }

    // and load it
    tileset::loadTileSetIndex(tsi_, *this);

    // make me read-only
    readOnly(true);
}

RemoteDriver::RemoteDriver(const boost::filesystem::path &root
                           , const OpenOptions &openOptions
                           , const RemoteOptions &options)
    : Driver(root, openOptions, options)
    , fetcher_(this->options().url, openOptions)
    , revision_()
{
    // asynchronous in nature
    capabilities().async = true;

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
    , Driver(root, cloneOptions.openOptions(), options, cloneOptions.mode())
    , fetcher_(this->options().url, cloneOptions.openOptions())
{
    // asynchronous in nature
    capabilities().async = true;

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

    // clone registry if exists
    if (auto registry = src.input(File::registry, NullWhenNotFound)) {
        copyFile(registry, output(File::registry));
    }

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

IStream::pointer RemoteDriver::input_impl(File type, const NullWhenNotFound_t&)
    const
{
    auto path(root() / filePath(type));
    LOG(info1) << "Loading from " << path << ".";
    return fileIStream(type, path, NullWhenNotFound);
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

IStream::pointer RemoteDriver::input_impl(const TileId &tileId
                                          , TileFile type
                                          , const NullWhenNotFound_t&)
    const
{
    return fetcher_.input(tileId, type, revision_, false);
}

void RemoteDriver::input_impl(const TileId &tileId, TileFile type
                              , const InputCallback &cb
                              , const IStream::pointer *notFound)
    const
{
    return fetcher_.input(tileId, type, revision_, cb, notFound);
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
    // TODO: implement me
    (void) tileId;
    (void) type;
    return {};
}

void RemoteDriver::stat_impl(const TileId &tileId, TileFile type
                             , const StatCallback &cb) const
{
    // TODO: implement me
    (void) tileId;
    (void) type;
    runCallback([&]() { return FileStat(); }, cb);
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

boost::any RemoteOptions::relocate(const RelocateOptions &options
                                   , const std::string &prefix) const
{
    auto res(options.apply(url));

    if (res.replacement) {
        // update
        if (options.dryRun) {
            LOG(info3)
                << prefix << "Would relocate URL <" << url << "> to URL <"
                << *res.replacement << ">.";
            return {};
        }

        auto out(*this);
        LOG(info3)
            << prefix << "Relocating URL <" << url << "> to URL <"
            << *res.replacement << ">.";
        out.url = *res.replacement;
        return out;
    }

    LOG(info3) << prefix << "Nothing to do with URL <" << url << ">.";
    return {};
}


bool RemoteDriver::reencode(const boost::filesystem::path&
                            , const RemoteOptions&
                            , const ReencodeOptions &options
                            , const std::string&)
{
    return !options.dryRun && !options.cleanup;
}

} } } // namespace vtslibs::vts::driver
