#ifndef vadstena_libs_tilestorage_merge_hpp_included_
#define vadstena_libs_tilestorage_merge_hpp_included_

#include "./types.hpp"

namespace vadstena { namespace tilestorage {

constexpr int MERGE_NO_FALLBACK_TILE = -100;

/** Merge tiles based on their quality. Areas not covered by any tile in `tiles`
 *  is covered by `fallbackQuad` quadrant of `fallback` tile.
 *
 * Prerequisities:
 *     * all input tiles have dimension AxA
 *     * fallback tile has dimension 2Ax2A
 *     * output tile has dimension AxA
 *
 * \param tileSize size of tile
 * \param tiles tiles to merge
 * \param fallback fallback tile for areas not covered by any tile
 *                 (fallback tile size = 2 * tileSize)
 * \param fallbackQuad which quadrant of fallback tile (or whole tile) to use:
 *            * 0: lower-left
 *            * 1: lower-right
 *            * 2: upper-left
 *            * 3: upper-right
 *            * other negative: ignore fallback tile
 * \return merged tile
 */
Tile merge(long tileSize, const Tile::list &tiles
           , const Tile &fallback, int fallbackQuad);

} } // namespace vadstena::tilestorage

#endif // vadstena_libs_tilestorage_merge_hpp_included_
