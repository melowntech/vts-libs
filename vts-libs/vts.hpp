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
#ifndef vtslibs_vts_hpp_included_
#define vtslibs_vts_hpp_included_

#include <memory>
#include <cmath>
#include <list>
#include <string>
#include <array>

#include <boost/optional.hpp>
#include <boost/noncopyable.hpp>
#include <boost/filesystem/path.hpp>

#include "utility/runnable.hpp"

#include "storage/lod.hpp"
#include "storage/range.hpp"

#include "vts/options.hpp"
#include "vts/tileset.hpp"
#include "vts/storage.hpp"
#include "vts/storageview.hpp"

namespace vtslibs { namespace vts {

enum class DatasetType {
    Unknown, TileSet, Storage, StorageView, TileIndex
};

DatasetType datasetType(const boost::filesystem::path &path);

DatasetType datasetType(const boost::filesystem::path &path
                        , const std::string &mime);

TileSet createTileSet(const boost::filesystem::path &path
                      , const TileSetProperties &properties
                      , CreateMode mode = CreateMode::failIfExists);

TileSet openTileSet(const boost::filesystem::path &path
                    , const OpenOptions &openOptions = OpenOptions());

TileSet cloneTileSet(const boost::filesystem::path &path, const TileSet &src
                     , const CloneOptions &cloneOptions);

TileSet concatTileSets(const boost::filesystem::path &path
                       , const std::vector<boost::filesystem::path> &tilesets
                       , const CloneOptions &createOptions);

Storage openStorage(const boost::filesystem::path &path
                    , OpenMode mode = OpenMode::readOnly
                    , const StorageLocker::pointer &locker = nullptr);

Storage createStorage(const boost::filesystem::path &path
                      , const StorageProperties &properties
                      , CreateMode mode
                      , const StorageLocker::pointer &locker = nullptr);

StorageView openStorageView(const boost::filesystem::path &path);

/** These flags can be passed via createOptions.createFlags() to
 *  aggregateTileSets.
 */
struct AggregateFlags { enum : CloneOptions::CreateFlags {
    dontAbsolutize = 0x1
    , sourceReferencesInMetatiles = 0x2
}; };

/** Creates aggreagated tileset from storage subset.
 */
TileSet aggregateTileSets(const boost::filesystem::path &path
                          , const boost::filesystem::path &storagePath
                          , const CloneOptions &createOptions
                          , const TilesetIdSet &tilesets);

/** Creates aggreagated tileset from storage subset.
 */
TileSet aggregateTileSets(const boost::filesystem::path &path
                          , const Storage &storage
                          , const CloneOptions &createOptions
                          , const TilesetIdList &tilesets);

/** Creates aggreagated tileset from storage view.
 */
TileSet aggregateTileSets(const boost::filesystem::path &path
                          , const StorageView &storageView
                          , const CloneOptions &createOptions);

TileSet aggregateTileSets(const boost::filesystem::path &path
                          , const Storage &storage
                          , const CloneOptions &co
                          , const TilesetIdSet &tilesets);

/** Creates aggreagated in-memory tileset from storage subset.
 */
TileSet aggregateTileSets(const Storage &storage
                          , const CloneOptions &createOptions
                          , const TilesetIdList &tilesets);

TileSet aggregateTileSets(const Storage &storage
                          , const CloneOptions &co
                          , const TilesetIdSet &tilesets);

/** Creates adapter for remote (HTTP) tileset.
 */
TileSet createRemoteTileSet(const boost::filesystem::path &path
                            , const std::string &url
                            , const CloneOptions &createOptions);

/** Creates adapter for local (filesystem) tileset.
 */
TileSet createLocalTileSet(const boost::filesystem::path &path
                           , const boost::filesystem::path &localPath
                           , const CloneOptions &createOptions);

// Asynchronous open support.

/** Base of all async open callbacks.
 */
struct OpenCallbackBase {
    virtual ~OpenCallbackBase() {}

    /** Called when async open fails.
     */
    virtual void error(const std::exception_ptr &exc) = 0;
};

/** Async storage open callback.
 */
struct StorageOpenCallback : OpenCallbackBase {
    typedef std::shared_ptr<StorageOpenCallback> pointer;
    virtual void done(Storage storage) = 0;
    virtual ~StorageOpenCallback() {}
};

/** Async storage view open callback.
 */
struct StorageViewOpenCallback : OpenCallbackBase {
    typedef std::shared_ptr<StorageViewOpenCallback> pointer;
    virtual ~StorageViewOpenCallback() {}

    virtual void done(StorageView storageView) = 0;

    /** Called by async machinery to open storage.
     */
    virtual void openStorage(const boost::filesystem::path &path
                             , const StorageOpenCallback::pointer
                             &callback) = 0;
};

/** Async tileset driver open callback.
 */
struct DriverOpenCallback : OpenCallbackBase {
    typedef std::shared_ptr<DriverOpenCallback> pointer;
    virtual ~DriverOpenCallback() {}

    virtual void done(std::shared_ptr<Driver> driver) = 0;

    /** Called by async machinery to open storage.
     */
    virtual void openStorage(const boost::filesystem::path &path
                             , const StorageOpenCallback::pointer
                             &callback) = 0;

    /** Called by async machinery to open another tileset driver.
     */
    virtual void openDriver(const boost::filesystem::path &path
                            , const OpenOptions &openOptions
                            , const DriverOpenCallback::pointer
                            &callback) = 0;
};

void openStorageView(const boost::filesystem::path &path
                     , const StorageViewOpenCallback::pointer &callback);

void openTilesetDriver(const boost::filesystem::path &path
                       , const OpenOptions &openOptions
                       , const DriverOpenCallback::pointer &callback);

} } // namespace vtslibs::vts

#endif // vtslibs_vts_hpp_included_
