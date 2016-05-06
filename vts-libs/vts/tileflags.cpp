#include "./tileflags.hpp"

namespace vadstena { namespace vts {

std::vector<TileFlags::TileFlag> TileFlags::mapping = {
    TileFlag(TileIndex::Flag::mesh, "mesh")
    , TileFlag(TileIndex::Flag::watertight, "watertight")
    , TileFlag(TileIndex::Flag::atlas, "atlas")
    , TileFlag(TileIndex::Flag::navtile, "navtile")
    , TileFlag(TileIndex::Flag::reference, "reference")
};

} } // namespace vadstena::vts
