/**
 * \file tilestorage.hpp
 * \author Vaclav Blazek <vaclav.blazek@citationtech.net>
 *
 * Tile storage manipulation.
 *
 * NB: tile set is specified by simple URI: TYPE:LOCATION where:
 *     TYPE     is type of backing storage (i.e. access driver to use);
 *              defaults to "flat"
 *     LOCATION is type-specific location of storage (e.g. root directory for
 *              filesystem based backing)
 */

#ifndef vadstena_libs_tilestorage_hpp_included_
#define vadstena_libs_tilestorage_hpp_included_

#include <memory>
#include <cmath>
#include <stdexcept>
#include <string>
#include <array>

#include <boost/filesystem/path.hpp>

#include <opencv2/core/core.hpp>

#include "math/geometry.hpp"
#include "geometry/parse-obj.hpp"

#include "./ids.hpp"

#include "./tilestorage/error.hpp"
#include "./tilestorage/types.hpp"
#include "./tilestorage/tileset.hpp"
#include "./tilestorage/storage.hpp"

namespace vadstena { namespace tilestorage {

/** Creates new storage.
 *
 * \param root storage root
 * \param properties properties to initialize output tile set with
 * \param mode what to do when storage already exists:
 *                 * failIfExists: storage must not exists prior this call
 *                 * overwrite: new storage is created
 * \return interface to new storage
 * \throws Error if storage cannot be created
 */
Storage::pointer createStorage(const boost::filesystem::path &root
                               , const StorageCreateProperties &properties
                               , CreateMode mode = CreateMode::failIfExists);

/** Opens existing storage.
 *
 * \param root storage root
 * \param mode what operations are allowed on storage:
 *                 * readOnly: only getters are allowed
 *                 * readWrite: both getters and setters are allowed
 * \return interface to new storage
 * \throws Error if storage cannot be opened
 */
Storage::pointer openStorage(const boost::filesystem::path &root
                             , OpenMode mode = OpenMode::readOnly);

/** Creates new tile set.
 *
 * \param locator locator that specifies tile set type and location.
 * \param properties properties to initialize new tile set with
 * \param mode what to do when tile set already exists:
 *                 * failIfExists: tile set must not exists prior this call
 *                 * overwrite: new tile set is created
 * \return interface to new tile set
 * \throws Error if tile set cannot be created
 */
TileSet::pointer createTileSet(const Locator &locator
                               , const CreateProperties &properties
                               , CreateMode mode = CreateMode::failIfExists);

/** Opens existing tile set.
 *
 * \param locator locator that specifies tile set type and location.
 * \param mode what operations are allowed on tile set:
 *                 * readOnly: only getters are allowed
 *                 * readWrite: both getters and setters are allowed
 * \return interface to new tile set
 * \throws Error if tile set cannot be opened
 */
TileSet::pointer openTileSet(const Locator &locator
                             , OpenMode mode = OpenMode::readOnly);

/** Clones existing tile set to a new tile set.
 *
 * \param locator locator that specifies tile set type and location.
 * \param srcLocator locator of source tile set
 * \param mode what to do when tile set already exists:
 *                 * failIfExists: tile set must not exists prior this call
 *                 * overwrite: new tile set is created
 * \return interface to new tile set
 * \throws Error if tile set cannot be opened
 */
TileSet::pointer cloneTileSet(const Locator &locator
                              , const Locator &srcLocator
                              , CreateMode mode = CreateMode::failIfExists);

/** Clones existing tile set to a new tile set.
 *
 * \param locator locator that specifies tile set type and location.
 * \param src source tile set
 * \param mode what to do when tile set already exists:
 *                 * failIfExists: tile set must not exists prior this call
 *                 * overwrite: new tile set is created
 * \return interface to new tile set
 * \throws Error if tile set cannot be opened
 */
TileSet::pointer cloneTileSet(const Locator &locator
                              , const TileSet::pointer &src
                              , CreateMode mode = CreateMode::failIfExists);

/** Clones existing tile set another existing tile set.
 *
 * \param dst destination tile set
 * \param src source tile set
 * \param mode what to do when dst tile set is not empty
 *                 * failIfExists: tile set be empty prior this call
 *                 * overwrite: tile set is overwritten
 * \return dst
 * \throws Error if tile set cannot be opened
 */
TileSet::pointer cloneTileSet(const TileSet::pointer &dst
                              , const TileSet::pointer &src
                              , CreateMode mode = CreateMode::failIfExists);

/** This function pastes tiles from one or more tile sets into existing tile
 *  set. Tile data are copied into result and metadata are generated
 *  accordingly.
 *
 * This operation is simplified merge of tile sets. Be aware that "last tile
 * wins" strategy is used: if there are more than one tiles with the same tileId
 * the last one (from last tile set specified on the command line) is placed
 * into the result.

 * To be used only to glue together non-overlaping data sets like:
 *   * webexports from one scene's targets
 *   * piecewise generated heightfield
 *
 * \param dst destination tile set
 * \param src source tile sets
 */
void pasteTileSets(const TileSet::pointer &dst
                   , const TileSet::list &src);

} } // namespace vadstena::tilestorage

#endif // vadstena_libs_tilestorage_hpp_included_
