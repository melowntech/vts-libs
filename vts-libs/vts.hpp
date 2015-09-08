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

#include "../storage/lod.hpp"
#include "../storage/range.hpp"

#include "./basetypes.hpp"
#include "./mesh.hpp"
#include "./metatile.hpp"
#include "./atlas.hpp"

namespace vadstena { namespace vts {

/**
 * \file vts/tileset.hpp
 * \author Vaclav Blazek <vaclav.blazek@citationtech.net>
 *
 * Tile set access.
 */

#ifndef vadstena_libs_vts_tileset_hpp_included_
#define vadstena_libs_vts_tileset_hpp_included_

#include <memory>
#include <cmath>
#include <list>
#include <string>
#include <array>

#include <boost/optional.hpp>
#include <boost/noncopyable.hpp>
#include <boost/filesystem/path.hpp>

#include "utility/runnable.hpp"

#include "../storage/lod.hpp"
#include "../storage/range.hpp"

#include "./properties.hpp"
#include "./basetypes.hpp"
#include "./mesh.hpp"
#include "./metatile.hpp"
#include "./atlas.hpp"

namespace vadstena { namespace vts {

TileSet createTileSet(const boost::filesystem::path &path
                      , const CreateProperties &properties
                      , CreateMode mode = CreateMode::failIfExists);

TileSet openTileSet(const boost::filesystem::path &path
                    , OpenMode mode = OpenMode::readOnly);

} } // namespace vadstena::vts

#endif // vadstena_libs_vts_hpp_included_
