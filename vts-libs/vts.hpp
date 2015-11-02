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

namespace vadstena { namespace vts {

TileSet createTileSet(const boost::filesystem::path &path
                      , const TileSetProperties &properties
                      , CreateMode mode = CreateMode::failIfExists);

TileSet openTileSet(const boost::filesystem::path &path);

class CloneOptions {
public:
    CloneOptions()
        : mode_(CreateMode::failIfExists)
        , allowDanglingNavtiles_(false)
    {}

    CreateMode mode() const { return mode_; }
    CloneOptions& mode(CreateMode mode) { mode_ = mode; return *this; }

    bool allowDanglingNavtiles() const { return allowDanglingNavtiles_; }
    CloneOptions& allowDanglingNavtiles(bool allowDanglingNavtiles) {
        allowDanglingNavtiles_ = allowDanglingNavtiles; return *this;
    }

    boost::optional<std::string> tilesetId() const {
        return tilesetId_;
    }
    CloneOptions& tilesetId(boost::optional<std::string> tilesetId) {
        tilesetId_ = tilesetId; return *this;
    }

private:
    CreateMode mode_;
    bool allowDanglingNavtiles_;
    boost::optional<std::string> tilesetId_;
};

TileSet cloneTileSet(const boost::filesystem::path &path, const TileSet &src
                     , const CloneOptions &cloneOptions);

Storage openStorage(const boost::filesystem::path &path
                    , OpenMode mode = OpenMode::readOnly);

Storage createStorage(const boost::filesystem::path &path
                      , const StorageProperties &properties
                      , CreateMode mode);

} } // namespace vadstena::vts

#endif // vadstena_libs_vts_hpp_included_
