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
#include <set>
#include <map>
#include <memory>

#include <boost/noncopyable.hpp>
#include <boost/format.hpp>

#include "utility/runnable.hpp"

#include "imgproc/png.hpp"

#include "../../../registry.hpp"
#include "../../../storage/sstreams.hpp"

#include "../../io.hpp"
#include "../../2d.hpp"
#include "../../tileset.hpp"
#include "../../debug.hpp"
#include "../../qtree-rasterize.hpp"

#include "../delivery.hpp"
#include "../driver.hpp"
#include "../tilesetindex.hpp"
#include "../config.hpp"

#include "runcallback.hpp"

namespace vtslibs { namespace vts {

namespace fs = boost::filesystem;
namespace vs = vtslibs::storage;

namespace {

std::shared_ptr<tileset::Index>
indexFromDriver(const FullTileSetProperties &properties
                , const Driver::pointer &driver)
{
    if (auto *i = driver->getTileIndex()) {
        // driver provides tile index, reuse with empty deleter
        return { i, [](void*) {} };
    }

    // no tileindex available -> load
    auto index(std::make_shared<tileset::Index>
               (registry::system.referenceFrames
                (properties.referenceFrame).metaBinaryOrder));

    tileset::loadTileSetIndex(*index, *driver);
    return index;
}

std::string filename(const fs::path &root, const TileId &tileId, TileFile type)
{
    return (root / str(boost::format("%s.%s") % tileId % type)).string();
}

IStream::pointer meta2d(const Driver &driver
                        , const tileset::Index &index
                        , const TileId &tileId
                        , bool noSuchFile)
{
    if (!Meta2d::isMetaId(tileId)) {
        if (noSuchFile) {
            LOGTHROW(err1, storage::NoSuchFile)
                << "Tile ID " << tileId << " is not valid for 2d metatile.";
        }
        return {};
    }

    // generate mask image from tileindex, serialize it as a png and wrap in
    // input stream
    return vs::memIStream(TileFile::mask
                          , imgproc::png::serialize
                          (meta2d(index.tileIndex, tileId), 9)
                          , driver.lastModified()
                          , filename(driver.root(), tileId, TileFile::meta2d));
}

void meta2d(const Driver &driver, const tileset::Index &index
            , const TileId &tileId, const InputCallback &cb)
{
    // no driver access, can be called immediately
    runCallback([&]() { return meta2d(driver, index, tileId, true); }, cb);
}

namespace constants {

const auto EmptyMask = imgproc::png::serialize(emptyDebugMask(), 9);

} // namespace constants

IStream::pointer maskStream(const IStream::pointer &is
                            , const Driver::pointer &pdriver
                            , const TileId &tileId, bool debug)
{
    const auto &driver(*pdriver);

    if (!is) {
        if (debug) {
            // no data and debug mode -> return transparent tile
            return vs::memIStream
                (TileFile::mask, constants::EmptyMask
                 , -1, filename(driver.root(), tileId, TileFile::mask));
        }

        // not found
        return {};
    }

    // generate mask image from mask, serialize it as a png and wrap in input
    // stream

    const auto meshMask(loadMeshMask(is));
    const auto png(debug
                   ? imgproc::png::serialize
                   (debugMask(meshMask, driver.capabilities().flattener), 9)
                   : imgproc::png::serialize
                   (mask2d(meshMask, driver.capabilities().flattener), 9));
    return vs::memIStream(TileFile::mask
                          , png, is->stat().lastModified
                          , filename(driver.root(), tileId, TileFile::mask));
}

IStream::pointer mask(const Driver::pointer &driver, const TileId &tileId
                      , FileFlavor flavor, bool noSuchFile)
{
    const auto debug(flavor == FileFlavor::debug);

    return maskStream((noSuchFile && !debug)
                      ? driver->input(tileId, TileFile::mesh)
                      : driver->input(tileId, TileFile::mesh, NullWhenNotFound)
                      , driver, tileId, debug);
}

void mask(const Driver::pointer &driver, const TileId &tileId
          , FileFlavor flavor, const InputCallback &cb)
{
    const auto debug(flavor == FileFlavor::debug);

    driver->input(tileId, TileFile::mesh, [=](const EIStream &eis)
        mutable -> void
    {
        if (const auto &is = eis.get(cb, utility::ExpectedAsSink{})) {
            runCallback([&]() {
                return maskStream(is, driver, tileId, debug);
            }, cb);
        }
    });
}

IStream::pointer meta(const Driver &driver, const tileset::Index &index
                      , const FullTileSetProperties&
                      , const TileId &tileId, FileFlavor flavor
                      , bool noSuchFile)
{
    if (flavor != FileFlavor::debug) {
        return (noSuchFile
                ? driver.input(tileId, TileFile::meta)
                : driver.input(tileId, TileFile::meta, NullWhenNotFound));
    }

    // generate debug metanode
    const auto debugNode(getNodeDebugInfo(index.tileIndex, tileId));

    std::ostringstream os;
    saveDebug(os, debugNode);
    return vs::memIStream("application/json; charset=utf-8"
                          , os.str(), -1 // where to get last modified?
                          , filename(driver.root(), tileId, TileFile::meta));
}

void meta(const Driver &driver, const tileset::Index &index
          , const FullTileSetProperties&
          , const TileId &tileId, FileFlavor flavor
          , const InputCallback &cb)
{
    if (flavor != FileFlavor::debug) {
        return driver.input(tileId, TileFile::meta, cb);
    }

    // generate debug metanode from tile index
    const auto debugNode(getNodeDebugInfo(index.tileIndex, tileId));

    std::ostringstream os;
    saveDebug(os, debugNode);

    cb(vs::memIStream("application/json; charset=utf-8"
                      , os.str(), -1 // where to get last modified?
                      , filename(driver.root(), tileId, TileFile::meta)));
}

/** TODO: make asynchronous version that launches metatile fetch for all
 *  metatiles needed and collects info in callback.
 *
 *  Since 2D interface was never used... do we need it?
 */
bool creditsFromMetatiles(const Driver &driver
                          , const tileset::Index &index
                          , const TileId &creditsId
                          , bool noSuchFile
                          , registry::IdSet &credits
                          , std::size_t maxCount
                          , std::time_t &lastModified)
{
    auto metaOrder(index.metaBinaryOrder());
    if (CreditTile::binaryOrder < metaOrder) {
        LOGTHROW(err3, storage::Error)
            << "unimplemented: cannot create credit tile from bigger "
            "3D metatiles";
    }

    int depthDiff(CreditTile::binaryOrder - metaOrder);
    int count(1 << depthDiff);
    int skip(1 << metaOrder);
    for (int j(0); j < count; ++j) {
        int row(creditsId.y + j * skip);
        for (int i(0); i < count; ++i) {
            TileId metaId(creditsId.lod, creditsId.x + i * skip, row);
            if (!index.meta(metaId)) { continue; }

            auto is(noSuchFile
                    ? driver.input(metaId, TileFile::meta)
                    : driver.input(metaId, TileFile::meta, NullWhenNotFound));
            if (!is) { return false; }
            loadCreditsFromMetaTile(is->get(), credits, is->name());

            // update last modified
            auto lm(is->stat().lastModified);
            if (lm > lastModified) { lastModified = lm; }
            is->close();

            // all credits seen
            if (credits.size() >= maxCount) { return true; }
        }
    }

    return true;
}

IStream::pointer credits(const Driver &driver
                         , const tileset::Index &index
                         , const FullTileSetProperties &properties
                         , const TileId &tileId
                         , bool noSuchFile)
{
    if (!CreditTile::isCreditId(tileId)) {
        if (noSuchFile) {
            LOGTHROW(err1, storage::NoSuchFile)
                << "Tile ID " << tileId << " is not valid for 2d credit tile.";
        }
        return {};
    }

    CreditTile tile;

    std::time_t lastModified(-1);

    auto extract([&](const registry::IdSet &credits)
    {
        for (const auto &cid : credits) {
            if (const auto *credit = registry::system.credits
                (cid, std::nothrow))
            {
                tile.credits.set(credit->id, boost::none);
            }
        }
    });

    if (properties.credits.size() <= 1) {
        extract(properties.credits);
    } else {
        registry::IdSet credits;
        if (!creditsFromMetatiles(driver, index, CreditTile::creditsId(tileId)
                                  , noSuchFile, credits
                                  , properties.credits.size()
                                  , lastModified))
        {
            return {};
        }

        extract(credits);
        if (lastModified <= 0) {
            lastModified = driver.lastModified();
        }
    }

    auto s(std::make_shared<StringIStream>
           (TileFile::credits
            , filename(driver.root(), tileId, TileFile::credits)
            , lastModified));

    // serialize credit tile
    saveCreditTile(s->sink(), tile, false);
    s->updateSize();

    // done
    return s;
}

IStream::pointer filterConfig(const IStream::pointer &raw)
{
    // load config and reset driver
    auto props(tileset::loadConfig(raw->get(), raw->name()));
    props.driverOptions = boost::any();
    auto s(std::make_shared<storage::StringIStream>(*raw));
    tileset::saveConfig(s->sink(), props);
    s->updateSize();
    return s;
}

} // namespace

Delivery::Delivery(AccessToken, const boost::filesystem::path &root
                   , const OpenOptions &openOptions)
    : driver_(Driver::open(root, openOptions))
    , properties_(tileset::loadConfig(*driver_))
    , index_(indexFromDriver(properties_, driver_))
{}

Delivery::Delivery(AccessToken, std::shared_ptr<Driver> driver)
    : driver_(std::move(driver))
    , properties_(tileset::loadConfig(*driver_))
    , index_(indexFromDriver(properties_, driver_))
{}

Delivery::pointer
Delivery::open(const boost::filesystem::path &root
               , const OpenOptions &openOptions)
{
    return std::make_shared<Delivery>(AccessToken{}, root, openOptions);
}

Delivery::pointer
Delivery::open(std::shared_ptr<Driver> driver)
{
    return std::make_shared<Delivery>(AccessToken{}, std::move(driver));
}

IStream::pointer Delivery::input(File type) const
{
    switch (type) {
    case File::config:
        return filterConfig(driver_->input(type));
    default: break;
    }
    return driver_->input(type);
}

IStream::pointer Delivery::input(File type, const NullWhenNotFound_t&) const
{
    switch (type) {
    case File::config:
        return filterConfig(driver_->input(type, NullWhenNotFound));
    default: break;
    }
    return driver_->input(type, NullWhenNotFound);
}

namespace {

inline void fixmeAsync() {
    LOGTHROW(err4, std::runtime_error)
        << "FIXME: make me async!";
}

} // namespace

IStream::pointer Delivery::input(const TileId &tileId, TileFile type
                                 , FileFlavor flavor) const
{
    switch (type) {
    case TileFile::meta2d:
        return meta2d(*driver_, *index_, tileId, true);

    case TileFile::mask:
        return mask(driver_, tileId, flavor, true);

    case TileFile::credits:
        return credits(*driver_, *index_, properties_, tileId, true);

    case TileFile::meta:
        return meta(*driver_, *index_, properties_, tileId, flavor, true);

    default:
        return driver_->input(tileId, type);
    }
}

IStream::pointer Delivery::input(const TileId &tileId, TileFile type
                                 , FileFlavor flavor
                                 , const NullWhenNotFound_t&) const
{
    switch (type) {
    case TileFile::meta2d:
        return meta2d(*driver_, *index_, tileId, false);

    case TileFile::mask:
        return mask(driver_, tileId, flavor, false);

    case TileFile::credits:
        return credits(*driver_, *index_, properties_, tileId, false);

    case TileFile::meta:
        return meta(*driver_, *index_, properties_, tileId, flavor, false);

    default:
        return driver_->input(tileId, type, NullWhenNotFound);
    }
}

void Delivery::input(const TileId &tileId, TileFile type, FileFlavor flavor
                     , const InputCallback &cb) const
{
    switch (type) {
    case TileFile::meta2d:
        return meta2d(*driver_, *index_, tileId, cb);

    case TileFile::mask:
        return mask(driver_, tileId, flavor, cb);

    case TileFile::credits:
        fixmeAsync();
        return cb(credits(*driver_, *index_, properties_, tileId, true));

    case TileFile::meta:
        return meta(*driver_, *index_, properties_, tileId, flavor, cb);

    default:
        // forward
        return driver_->input(tileId, type, cb);
    }
}

FileStat Delivery::stat(File type) const
{
    return driver_->stat(type);
}

FileStat Delivery::stat(const TileId &tileId, TileFile type) const
{
    return driver_->stat(tileId, type);
}

void Delivery::stat(const TileId &tileId, TileFile type
                    , const StatCallback &cb) const
{
    return driver_->stat(tileId, type, cb);
}

IStream::pointer Delivery::input(const std::string &name) const
{
    return driver_->input(name);
}

IStream::pointer Delivery::input(const std::string &name
                                 , const NullWhenNotFound_t&) const
{
    return driver_->input(name, NullWhenNotFound);
}

FileStat Delivery::stat(const std::string &name) const
{
    return driver_->stat(name);
}

Resources Delivery::resources() const
{
    return driver_->resources();
}

bool Delivery::externallyChanged() const
{
    return driver_->externallyChanged();
}

std::time_t Delivery::lastModified() const
{
    return driver_->lastModified();
}

MapConfig Delivery::mapConfig(bool includeExtra) const
{
    return TileSet::mapConfig(*driver_, includeExtra);
}

MeshTilesConfig Delivery::meshTilesConfig(bool includeExtra) const
{
    return TileSet::meshTilesConfig(*driver_, includeExtra);
}

} } // namespace vtslibs::vts
