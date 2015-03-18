#ifndef vadstena_libs_tilestorage_tileopext_hpp_included_
#define vadstena_libs_tilestorage_tileopext_hpp_included_

#include "./types.hpp"
#include "./tileop.hpp"

namespace vadstena { namespace tilestorage {

bool valid(const Tile &tile);

bool valid(const MetaNode &metanode);

inline bool valid(const Tile &tile)
{
    return tile.metanode.exists();
}

inline bool valid(const MetaNode &metanode)
{
    return metanode.exists();
}

} } // namespace vadstena::tilestorage

#endif // vadstena_libs_tilestorage_tileoext_hpp_included_
