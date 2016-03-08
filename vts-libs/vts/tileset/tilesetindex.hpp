/**
 * \file vts/tilesetindex.hpp
 * \author Vaclav Blazek <vaclav.blazek@citationtech.net>
 *
 * Tile set index access.
 */

#ifndef vadstena_libs_vts_tileset_tilesetindex_hpp_included_
#define vadstena_libs_vts_tileset_tilesetindex_hpp_included_

#include "../tileindex.hpp"
#include "./driver.hpp"

namespace vadstena { namespace vts { namespace tileset {

struct Index {
    /** Tile index (tile data presence flags)
     */
    TileIndex tileIndex;

    /** Tileset references.
     */
    TileIndex references;

    bool check(const TileId &tileId, TileFile type) const;

    bool real(const TileId &tileId) const;

    int getReference(const TileId &tileId) const;
};

void loadTileSetIndex(Index &tsi, Driver &driver);

void saveTileSetIndex(const Index &tsi, Driver &driver);

// inlines

inline bool Index::real(const TileId &tileId) const
{
    return check(tileId, TileFile::mesh);
}

} } } // namespace vadstena::vts::tileset

#endif // vadstena_libs_vts_tileset_detail_hpp_included_
