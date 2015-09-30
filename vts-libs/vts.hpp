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
                      , const StaticProperties &properties
                      , CreateMode mode = CreateMode::failIfExists);

TileSet openTileSet(const boost::filesystem::path &path);

} } // namespace vadstena::vts

#endif // vadstena_libs_vts_hpp_included_
