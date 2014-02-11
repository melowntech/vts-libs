#include "./merge.hpp"

namespace vadstena { namespace tilestorage {

Tile merge(const Tile::list &tiles, const Tile &fallback
           , int fallbackQuad)
{
    // TODO: implement me

    return tiles.front();
    (void) fallback;
    (void) fallbackQuad;
}

} } // namespace vadstena::tilestorage
