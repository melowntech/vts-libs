/**
 * \file vts/tilesetindex.hpp
 * \author Vaclav Blazek <vaclav.blazek@citationtech.net>
 *
 * Tile set index access.
 */

#ifndef vadstena_libs_vts_tileset_tilesetindex_hpp_included_
#define vadstena_libs_vts_tileset_tilesetindex_hpp_included_

#include "../tileindex.hpp"

namespace vadstena { namespace vts {

class Driver;

namespace tileset {

class Index {
public:
    Index(unsigned int metaBinaryOrder = 0)
        : metaBinaryOrder_(metaBinaryOrder)
    {}

    /** Tile index (tile data presence flags)
     */
    TileIndex tileIndex;

    /** Tileset references.
     */
    TileIndex references;

    bool check(const TileId &tileId, TileFile type) const;

    bool real(const TileId &tileId) const;

    int getReference(const TileId &tileId) const;

    bool meta(const TileId &tileId) const;

    TileIndex deriveMetaIndex() const;

private:
    unsigned int metaBinaryOrder_;
};

void loadTileSetIndex(Index &tsi, const Driver &driver);

void saveTileSetIndex(const Index &tsi, Driver &driver);

void loadTileSetIndex(Index &tsi, const boost::filesystem::path &path);

void saveTileSetIndex(const Index &tsi, const boost::filesystem::path &path);

// inlines

inline bool Index::real(const TileId &tileId) const
{
    return check(tileId, TileFile::mesh);
}

} } } // namespace vadstena::vts::tileset

#endif // vadstena_libs_vts_tileset_detail_hpp_included_
