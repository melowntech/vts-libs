#ifndef vtslibs_vts0_tileopext_hpp_included_
#define vtslibs_vts0_tileopext_hpp_included_

#include "./types.hpp"
#include "./tileop.hpp"

namespace vtslibs { namespace vts0 {

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

} } // namespace vtslibs::vts0

#endif // vtslibs_vts0_tileoext_hpp_included_
