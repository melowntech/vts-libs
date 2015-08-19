/**
 * \file vts.hpp
 * \author Vaclav Blazek <vaclav.blazek@citationtech.net>
 *
 * VTS (Vadstena Tile Set) manipulation.
 *
 */

#ifndef vadstena_libs_vts_hpp_included_
#define vadstena_libs_vts_hpp_included_

#include <memory>
#include <cmath>
#include <stdexcept>
#include <string>
#include <array>
#include <functional>

#include <boost/filesystem/path.hpp>

#include <opencv2/core/core.hpp>

#include "utility/runnable.hpp"

#include "math/geometry.hpp"
#include "geometry/parse-obj.hpp"

#include "./ids.hpp"

#include "./storage/error.hpp"
#include "./vts/types.hpp"
#include "./vts/tileset.hpp"

namespace vadstena { namespace vts {

/** Creates new tile set.
 *
 * \param path path to new tileset
 * \param properties properties to initialize new tile set with
 * \param mode what to do when tile set already exists:
 *                 * failIfExists: tile set must not exists prior this call
 *                 * overwrite: new tile set is created
 * \return interface to new tile set
 * \throws Error if tile set cannot be created
 */
TileSet::pointer createTileSet(const boost::filesystem::path &path
                               , const CreateProperties &properties
                               , CreateMode mode = CreateMode::failIfExists);

/** Opens existing tile set.
 *
 * \param path path to existing tileset
 * \param mode what operations are allowed on tile set:
 *                 * readOnly: only getters are allowed
 *                 * readWrite: both getters and setters are allowed
 * \return interface to new tile set
 * \throws Error if tile set cannot be opened
 */
TileSet::pointer openTileSet(const boost::filesystem::path &path
                             , OpenMode mode = OpenMode::readOnly);

} } // namespace vadstena::vts

#endif // vadstena_libs_vts_hpp_included_
