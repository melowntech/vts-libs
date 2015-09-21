#ifndef vadstena_libs_vts_types_hpp_included_
#define vadstena_libs_vts_types_hpp_included_

#include <string>

#include "math/geometry_core.hpp"

#include "basetypes.hpp"
#include "mesh.hpp"
#include "atlas.hpp"
#include "navtile.hpp"

namespace vadstena { namespace vts {

struct Tile {
    Mesh::pointer mesh;
    bool watertight;
    Atlas::pointer atlas;
    NavTile::pointer navtile;

    Tile() : watertight(true) {}
};

} } // namespace vadstena::vts

#endif // vadstena_libs_vts_types_hpp_included_
