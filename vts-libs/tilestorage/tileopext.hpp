#ifndef vtslibs_tilestorage_tileopext_hpp_included_
#define vtslibs_tilestorage_tileopext_hpp_included_

#include "./types.hpp"
#include "./tileop.hpp"

namespace vtslibs { namespace tilestorage {

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

} } // namespace vtslibs::tilestorage

#endif // vtslibs_tilestorage_tileoext_hpp_included_
