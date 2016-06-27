/**
 * \file vts/tilesetindex.hpp
 * \author Vaclav Blazek <vaclav.blazek@citationtech.net>
 *
 * Tile set index access.
 */

#ifndef vadstena_libs_vts_tileset_tilesetindex_hpp_included_
#define vadstena_libs_vts_tileset_tilesetindex_hpp_included_

#include <memory>

#include "../tileindex.hpp"

namespace vadstena { namespace vts {

class Driver;

namespace tileset {

class Index {
public:
    typedef std::shared_ptr<Index> pointer;

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

    bool real(const TileId &tileId, bool alien) const;

    int getReference(const TileId &tileId) const;

    bool meta(const TileId &tileId) const;

    TileIndex deriveMetaIndex() const;

    unsigned int metaBinaryOrder() const { return metaBinaryOrder_; }

private:
    unsigned int metaBinaryOrder_;
};

void loadTileSetIndex(Index &tsi, const Driver &driver);

void saveTileSetIndex(const Index &tsi, Driver &driver);

void loadTileSetIndex(Index &tsi, const boost::filesystem::path &path);

void saveTileSetIndex(const Index &tsi, const boost::filesystem::path &path);

void saveTileSetIndex(const Index &tsi, std::ostream &os);

Index::pointer loadTileSetIndex(const Driver &driver);

// inlines

inline bool Index::real(const TileId &tileId) const
{
    return tileIndex.real(tileId);
}

inline bool Index::real(const TileId &tileId, bool alien) const
{
    return tileIndex.real(tileId, alien);
}

} } } // namespace vadstena::vts::tileset

#endif // vadstena_libs_vts_tileset_detail_hpp_included_
