/**
 * \file registry/types.hpp
 * \author Vaclav Blazek <vaclav.blazek@citationtech.net>
 */

#ifndef vadstena_libs_registry_types_hpp_included_
#define vadstena_libs_registry_types_hpp_included_

#include <vector>
#include <string>

#include "math/geometry_core.hpp"

#include "../storage/range.hpp"

namespace vadstena { namespace registry {

using storage::Lod;
using storage::LodRange;
using storage::CreditId;
using storage::CreditIds;
typedef storage::Range<double> HeightRange;

typedef std::set<std::string> StringIdSet;
typedef std::set<int> IdSet;

typedef math::Extents2_<unsigned int> TileRange;

struct View {
    std::vector<std::string> surfaces;
    registry::StringIdSet boundLayers;
    registry::StringIdSet freeLayers;
};

struct Roi {
    std::string exploreUrl;
    std::string roiUrl;
    std::string thumbnailUrl;

    typedef std::vector<Roi> list;
};

} } // namespace vadstena::registry

#endif // vadstena_libs_registry_types_hpp_included_
