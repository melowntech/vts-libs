#ifndef vadstena_libs_tilestorage_merge_hpp_included_
#define vadstena_libs_tilestorage_merge_hpp_included_

#include <boost/optional.hpp>

#include "./types.hpp"

namespace vadstena { namespace tilestorage {

constexpr int MERGE_NO_FALLBACK_TILE = -100;

struct MergedTile : public Tile {
    MergedTile() : singleSource(false) {}
    MergedTile(const Tile &tile) : Tile(tile), singleSource(false) {}
    MergedTile(const Tile &tile, bool singleSource)
        : Tile(tile), singleSource(singleSource) {}

    /** Tile data originates from single tile if true.
     */
    bool singleSource;

    /** Returns prefered pixel size if available.
     */
    boost::optional<double> pixelSize() const;
};

/** Merge tiles based on their quality. Areas not covered by any tile in `tiles`
 *  is covered by `fallbackQuad` quadrant of `fallback` tile.
 *
 * Prerequisities:
 *     * all input tiles have dimension AxA
 *     * fallback tile has dimension 2Ax2A
 *     * output tile has dimension AxA
 *
 * \param tileId tile id (for diagnostics purposes only)
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
MergedTile merge(const TileId &tileId, long tileSize, const Tile::list &tiles
                 , const Tile &fallback, int fallbackQuad);

} } // namespace vadstena::tilestorage

#endif // vadstena_libs_tilestorage_merge_hpp_included_
