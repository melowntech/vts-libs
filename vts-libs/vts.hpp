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

#include "./vts/tileset.hpp"
#include "./vts/storage.hpp"
#include "./vts/storageview.hpp"

namespace vadstena { namespace vts {

enum class DatasetType { Unknown, TileSet, Storage, StorageView };

DatasetType datasetType(const boost::filesystem::path &path);

TileSet createTileSet(const boost::filesystem::path &path
                      , const TileSetProperties &properties
                      , CreateMode mode = CreateMode::failIfExists);

TileSet openTileSet(const boost::filesystem::path &path);

class CloneOptions {
public:
    CloneOptions()
        : mode_(CreateMode::failIfExists)
    {}

    CreateMode mode() const { return mode_; }
    CloneOptions& mode(CreateMode mode) { mode_ = mode; return *this; }

    boost::optional<std::string> tilesetId() const {
        return tilesetId_;
    }

    CloneOptions& tilesetId(boost::optional<std::string> tilesetId) {
        tilesetId_ = tilesetId; return *this;
    }

    boost::optional<LodRange> lodRange() const { return lodRange_; }

    CloneOptions& lodRange(const boost::optional<LodRange> &lodRange) {
        lodRange_ = lodRange;  return *this;
    }

private:
    CreateMode mode_;
    boost::optional<std::string> tilesetId_;
    boost::optional<LodRange> lodRange_;
};

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

} } // namespace vadstena::vts

#endif // vadstena_libs_vts_hpp_included_
