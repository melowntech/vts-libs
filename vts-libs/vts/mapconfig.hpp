/**
 * \file vts/mapconfig.hpp
 * \author Vaclav Blazek <vaclav.blazek@citationtech.net>
 */

#ifndef vadstena_libs_vts_mapconfig_hpp_included_
#define vadstena_libs_vts_mapconfig_hpp_included_

#include <iostream>

#include "../registry.hpp"
#include "./tileset/properties.hpp"

namespace vadstena { namespace vts {

struct MapConfig {
    registry::Srs::dict srs;
    registry::ReferenceFrame referenceFrame;
    registry::Credit::dict credits;
    registry::BoundLayer::dict boundLayers;
    registry::Position position;
};

/** Save map config into stream.
 */
void saveMapConfig(const MapConfig &mapConfig, std::ostream &os);

} } // namespace vadstena::vts

#endif // vadstena_libs_vts_mapconfig_hpp_included_
