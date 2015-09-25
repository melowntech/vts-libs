/**
 * \file vts/mapconfig.hpp
 * \author Vaclav Blazek <vaclav.blazek@citationtech.net>
 */

#ifndef vadstena_libs_vts_mapconfig_hpp_included_
#define vadstena_libs_vts_mapconfig_hpp_included_

#include <iostream>

#include <boost/filesystem/path.hpp>

#include "../registry.hpp"
#include "./tileset/properties.hpp"

namespace vadstena { namespace vts {

struct Surface {
    std::string id;
    boost::filesystem::path root;
    storage::LodRange lodRange;
    registry::TileRange tileRange;
    boost::optional<std::string> textureLayer;
    unsigned int revision;

    Surface() : revision(0) {}

    typedef std::vector<Surface> list;
};

struct View {
    registry::StringIdSet surfaces;
    registry::StringIdSet boundLayers;
    registry::StringIdSet freeLayers;
};

struct MapConfig {
    registry::Srs::dict srs;
    registry::ReferenceFrame referenceFrame;
    registry::Credit::dict credits;
    registry::BoundLayer::dict boundLayers;
    registry::Position position;
    bool textureAtlasReady;

    Surface::list surfaces;

    View view;

    static const char *contentType;

    MapConfig() : textureAtlasReady(false) {}
};

/** Save map config into stream.
 */
void saveMapConfig(const MapConfig &mapConfig, std::ostream &os);

} } // namespace vadstena::vts

#endif // vadstena_libs_vts_mapconfig_hpp_included_
