#ifndef vadstena_libs_vts_hpp_included_
#define vadstena_libs_vts_hpp_included_

#include <memory>
#include <cmath>
#include <list>
#include <string>
#include <array>

#include <boost/optional.hpp>
#include <boost/noncopyable.hpp>
#include <boost/filesystem/path.hpp>

#include "utility/runnable.hpp"

#include "./storage/lod.hpp"
#include "./storage/range.hpp"

#include "./vts/options.hpp"
#include "./vts/tileset.hpp"
#include "./vts/storage.hpp"
#include "./vts/storageview.hpp"

namespace vadstena { namespace vts {

enum class DatasetType {
    Unknown, TileSet, Storage, StorageView, TileIndex
};

DatasetType datasetType(const boost::filesystem::path &path);

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
                    , OpenMode mode = OpenMode::readOnly);

Storage createStorage(const boost::filesystem::path &path
                      , const StorageProperties &properties
                      , CreateMode mode);

StorageView openStorageView(const boost::filesystem::path &path);

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

} } // namespace vadstena::vts

#endif // vadstena_libs_vts_hpp_included_
