#ifndef vadstena_libs_vts_debug_hpp_included_
#define vadstena_libs_vts_debug_hpp_included_

#include "./tileindex.hpp"

namespace vadstena { namespace vts {

struct DebugNode {
    std::uint32_t indexFlags;
    std::uint32_t metaFlags;
};

DebugNode getNodeDebugInfo(const TileIndex &tileIndex, const TileId &tileId);

void saveDebug(std::ostream &out, const DebugNode &debugNode);

} } // namespace vadstena::vts

#endif // vadstena_libs_vts_debug_hpp_included_
