#ifndef vtslibs_vts_types_hpp_included_
#define vtslibs_vts_types_hpp_included_

#include <cstdint>
#include <string>

#include "math/geometry_core.hpp"

#include "../storage/credits.hpp"

#include "basetypes.hpp"
#include "geomextents.hpp"
#include "mesh.hpp"
#include "atlas.hpp"
#include "navtile.hpp"

namespace vtslibs { namespace vts {

struct Tile {
    Mesh::pointer mesh;
    Atlas::pointer atlas;
    NavTile::pointer navtile;
    GeomExtents geomExtents;

    // credits to store in metanode
    storage::CreditIds credits;

    bool alien;

    Tile& setAlien(bool value) {
        alien = value;
        return *this;
    }

    Tile() : alien(false) {}
};

} } // namespace vtslibs::vts

#endif // vtslibs_vts_types_hpp_included_
