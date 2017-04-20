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
#include "../../../storage/fstreams.hpp"
#include "../../io.hpp"
#include "../../config.hpp"
#include "../tilardriver.hpp"

namespace vtslibs { namespace vts0 {

namespace fs = boost::filesystem;

namespace {
    const std::string KeyBinaryOrder("binaryOrder");
    const std::uint8_t DefaultBinaryOrder(5);

    const std::string KeyUUID("uuid");

    const std::string ConfigName("config.json");
    const std::string TileIndexName("index.bin");
    const std::string TransactionRoot("tx");

    const std::string filePath(File type)
    {
        switch (type) {
        case File::config: return ConfigName;
        case File::tileIndex: return TileIndexName;
        default: break;
        }
        throw "unknown file type";
    }

    template <typename T, class Enable = void>
    T getOption(const DriverProperties::Options&, const std::string &
                , const boost::optional<T>& = boost::none);

    template
    <typename T
     , class = typename std::enable_if<std::is_unsigned<T>::value>::type>
    T getOption(const DriverProperties::Options &options
                , const std::string &key
                , const boost::optional<T> &defaultValue)
    {
        auto foptions(options.find(key));
        if (foptions == options.end()) {
            if (!defaultValue) {
                LOGTHROW(err2, std::runtime_error)
                    << "Option <" << key << "> not found "
                    "and no default value has been provided.";
            }
            return *defaultValue;
        }

        std::uint64_t value{};
        try {
            value = boost::any_cast<std::uint64_t>(foptions->second);
        } catch (const boost::bad_any_cast&) {
            LOGTHROW(err2, std::logic_error)
                << "Options value <" << key << "> is not an unsigned integer.";
        }

        if (value > std::numeric_limits<T>::max()) {
            LOGTHROW(err2, std::logic_error)
                << "Options value <" << key << "> is doesn't fit into "
                "requested type.";
        }
        return static_cast<T>(value);
    }

    template<>
    boost::uuids::uuid getOption<boost::uuids::uuid, void>
    (const DriverProperties::Options &options, const std::string &key
     , const boost::optional<boost::uuids::uuid> &defaultValue)
    {
        auto foptions(options.find(key));
        if (foptions == options.end()) {
            if (!defaultValue) {
                LOGTHROW(err2, std::runtime_error)
                    << "Option <" << key << "> not found "
                    "and no default value has been provided.";
            }
            return *defaultValue;
        }

        try {
            return boost::uuids::string_generator()
                (boost::any_cast<std::string>(foptions->second));
        } catch (const boost::bad_any_cast&) {
            LOGTHROW(err2, std::logic_error)
                << "Options value <" << key << "> is not a string.";
        }

        throw;
    }

    boost::uuids::uuid generateUuid() {
        // generate random uuid
        return boost::uuids::random_generator()();
    }
} // namespace

namespace tilardriver {

long mask(int order)
{
    long value(0);
    for (long bit(1l); order; --order, bit <<= 1) {
        value |= bit;
    }
    return value;
}

Options::Options(const Driver::CreateProperties &properties)
    : binaryOrder(getOption<decltype(binaryOrder)>
                 (properties->driver.options, KeyBinaryOrder))
    , uuid(getOption<boost::uuids::uuid>
           (properties->driver.options, KeyUUID))
    , tileMask(mask(binaryOrder))
{}

Options::Options(const Driver::CreateProperties &properties, bool)
    : binaryOrder(getOption<decltype(binaryOrder)>
                 (properties->driver.options, KeyBinaryOrder
                  , DefaultBinaryOrder))
    , uuid(properties.cloned
           ? generateUuid()
           : getOption<boost::uuids::uuid>
           (properties->driver.options, KeyUUID, generateUuid()))
    , tileMask(mask(binaryOrder))
{}

} // namespace tilardriver

TilarDriver::TilarDriver(const boost::filesystem::path &root
                         , CreateMode mode, const CreateProperties &properties)
    : Driver(false)
    , root_(absolute(root)), tmp_(root_ / TransactionRoot)
    , options_(properties, true)
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

TilarDriver::TilarDriver(const boost::filesystem::path &root
                         , OpenMode mode)
    : Driver(mode == OpenMode::readOnly)
    , root_(absolute(root)), tmp_(root_ / TransactionRoot)
    , mapConfigPath_(root_ / filePath(File::config))
    , options_(vts0::loadConfig(mapConfigPath_))
    , cache_(root_, options_, (mode == OpenMode::readOnly))
    , openStat_(FileStat::stat(mapConfigPath_))
{
}

TilarDriver::~TilarDriver()
{
    if (tx_) {
        LOG(warn3)
            << "Active transaction on driver close (" << root_
            << "); rolling back.";
        try {
            rollback_impl();
        } catch (const std::exception &e) {
            LOG(warn3)
                << "Error while trying to destroy active transaction on "
                "driver close: <" << e.what() << ">.";
        }
    }
}

OStream::pointer TilarDriver::output_impl(File type)
{
    const auto name(filePath(type));
    if (!tx_) {
        // no transaction -> plain file
        const auto path(root_ / name);
        LOG(info1) << "Saving to " << path << ".";
        return fileOStream(type, path);
    }

    const auto path(tmp_ / name);
    LOG(info1) << "Saving to " << path << ".";

    const auto tmpPath(utility::addExtension(path, ".tmp"));
    return fileOStream
        (type, tmpPath, [this, path, tmpPath, name](bool success)
    {
        if (!success) {
            // failed -> remove
            LOG(warn2)
                << "Removing failed file " << tmpPath << ".";
            fs::remove(tmpPath);
            return;
        }

        // OK -> move file to destination
        LOG(info1)
            << "Moving file " << tmpPath << " to " << path << ".";
        rename(tmpPath, path);

        // remember file in transaction
        tx_->files.insert(name);
});
}

IStream::pointer TilarDriver::input_impl(File type) const
{
    const auto name(filePath(type));

    auto path(root_ / name);
    if (tx_) {
        // we have active transaction: is file part of the tx?
        auto ffiles(tx_->files.find(name));
        if (ffiles != tx_->files.end()) {
            // yes -> tmp file
            path = tmp_ / name;
        }
    }

    LOG(info1) << "Loading from " << path << ".";
    return fileIStream(type, path);
}

OStream::pointer TilarDriver::output_impl(const TileId tileId, TileFile type)
{
    return cache_.output(tileId, type);
}

IStream::pointer TilarDriver::input_impl(const TileId tileId, TileFile type)
    const
{
    return cache_.input(tileId, type);
}

FileStat TilarDriver::stat_impl(File type) const
{
    // TODO: add tx support
    const auto name(filePath(type));
    const auto path(root_ / name);
    LOG(info1) << "Statting " << path << ".";
    return FileStat::stat(path);
}

FileStat TilarDriver::stat_impl(const TileId tileId, TileFile type) const
{
    return cache_.stat(tileId, type);
}

storage::Resources TilarDriver::resources_impl() const
{
    return cache_.resources();
}

void TilarDriver::remove_impl(const TileId tileId, TileFile type)
{
    return cache_.remove(tileId, type);
}

void TilarDriver::begin_impl()
{
    if (tx_) {
        LOGTHROW(err2, storage::PendingTransaction)
            << "Pending transaction.";
    }

    // remove whole tmp directory
    remove_all(tmp_);
    // create fresh tmp directory
    create_directories(tmp_);

    // begin tx
    tx_ = boost::in_place();
}

void TilarDriver::commit_impl()
{
    if (!tx_) {
        LOGTHROW(err2, storage::PendingTransaction)
            << "No pending transaction.";
    }

    for (const auto &file : tx_->files) {
        // move file
        rename(tmp_ / file, root_ / file);
    }

    // remove whole tmp directory
    remove_all(tmp_);
    cache_.commit();

    // no tx at all
    tx_ = boost::none;
}

void TilarDriver::rollback_impl()
{
    if (!tx_) {
        LOGTHROW(err2, storage::PendingTransaction)
            << "No pending transaction.";
    }

    // remove whole tmp directory
    remove_all(tmp_);
    cache_.rollback();

    // no tx at all
    tx_ = boost::none;
}

void TilarDriver::flush_impl()
{
    cache_.commit();
}

void TilarDriver::drop_impl()
{
    if (tx_) {
        LOGTHROW(err2, storage::PendingTransaction)
            << "Cannot drop tile set inside an active transaction.";
    }

    // remove whole tmp directory
    remove_all(root_);
}

bool TilarDriver::externallyChanged_impl() const
{
    return openStat_.changed(FileStat::stat(mapConfigPath_));
}

DriverProperties TilarDriver::properties_impl() const
{
    DriverProperties dp;
    dp.type = "tilar";
    dp.options[KeyBinaryOrder]
        = boost::any(std::uint64_t(options_.binaryOrder));
    dp.options[KeyUUID]
        = boost::any(to_string(options_.uuid));
    return dp;
}

const std::string TilarDriver::help
("Filesystem-based storage driver with deep directory structure. "
 "Tiles and metatiles are stored inside Tile Archives.");

} } // namespace vtslibs::vts0
