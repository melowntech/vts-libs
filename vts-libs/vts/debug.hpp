#ifndef vtslibs_vts_debug_hpp_included_
#define vtslibs_vts_debug_hpp_included_

#include "./tileindex.hpp"
#include "./mapconfig.hpp"

namespace vtslibs { namespace vts {

struct DebugNode {
    std::uint32_t indexFlags;
    std::uint32_t metaFlags;
};

DebugNode getNodeDebugInfo(const TileIndex &tileIndex, const TileId &tileId);

void saveDebug(std::ostream &out, const DebugNode &debugNode);

} } // namespace vtslibs::vts

#endif // vtslibs_vts_debug_hpp_included_
