#ifndef vadstena_libs_vts_tileopext_hpp_included_
#define vadstena_libs_vts_tileopext_hpp_included_

#include "./types.hpp"
#include "./tileop.hpp"

namespace vadstena { namespace vts {

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

} } // namespace vadstena::vts

#endif // vadstena_libs_vts_tileoext_hpp_included_
