#ifndef vadstena_libs_vts_types_hpp_included_
#define vadstena_libs_vts_types_hpp_included_

#include <cstdint>
#include <string>

#include "math/geometry_core.hpp"

#include "../storage/credits.hpp"

#include "basetypes.hpp"
#include "mesh.hpp"
#include "atlas.hpp"
#include "navtile.hpp"

namespace vadstena { namespace vts {

struct Tile {
    Mesh::pointer mesh;
    Atlas::pointer atlas;
    NavTile::pointer navtile;

    // credits to store in metanode
    storage::CreditIds credits;

    bool alien;

    Tile& setAlien(bool value) {
        alien = value;
        return *this;
    }

    Tile() : alien(false) {}
};

} } // namespace vadstena::vts

#endif // vadstena_libs_vts_types_hpp_included_
