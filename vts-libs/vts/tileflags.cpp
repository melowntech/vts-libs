#include "./tileflags.hpp"

namespace vadstena { namespace vts {

std::vector<TileFlags::TileFlag> TileFlags::mapping = {
    TileFlag(Match(TiFlag::mesh | TiFlag::alien, TiFlag::mesh), "mesh")
    , TileFlag(Match(TiFlag::alien | TiFlag::mesh
                     , TiFlag::alien | TiFlag::mesh), "alien")
    , TileFlag(Match(TiFlag::watertight, TiFlag::watertight), "watertight")
    , TileFlag(Match(TiFlag::atlas, TiFlag::atlas), "atlas")
    , TileFlag(Match(TiFlag::navtile, TiFlag::navtile), "navtile")
};

} } // namespace vadstena::vts
