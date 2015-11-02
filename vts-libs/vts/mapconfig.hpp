/**
 * \file vts/mapconfig.hpp
 * \author Vaclav Blazek <vaclav.blazek@citationtech.net>
 */

#ifndef vadstena_libs_vts_mapconfig_hpp_included_
#define vadstena_libs_vts_mapconfig_hpp_included_

#include <iostream>

#include <boost/filesystem/path.hpp>

#include "../registry.hpp"
#include "./basetypes.hpp"

namespace vadstena { namespace vts {

struct SurfaceCommonConfig {
    boost::filesystem::path root;
    storage::LodRange lodRange;
    registry::TileRange tileRange;
    boost::optional<std::string> textureLayer;
    unsigned int revision;

    SurfaceCommonConfig() : revision(0) {}
};

struct SurfaceConfig : SurfaceCommonConfig {
    TilesetId id;

    SurfaceConfig() {}

    typedef std::vector<SurfaceConfig> list;
};

struct GlueConfig : SurfaceCommonConfig {
    Glue::Id id;

    GlueConfig() {}
    GlueConfig(const SurfaceConfig &surface)
        : SurfaceCommonConfig(surface)
    {}

    typedef std::vector<GlueConfig> list;
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

    SurfaceConfig::list surfaces;
    GlueConfig::list glues;

    View view;

    static const char *contentType;

    MapConfig() : textureAtlasReady(false) {}

    /** Merges in mapConfig for one tileset.
     *
     * \param tilesetMapConfig single tileset mapConfig
     * \param root path to tileset
     */
    void mergeTileSet(const MapConfig &tilesetMapConfig
                      , const boost::filesystem::path &root);

    /** Merges in mapConfig for one tileset as a glue.
     *
     * \param glueMapConfig single tileset mapConfig
     * \param glue glue definition
     * \param root path to glues
     *
     * NB: glue path is composed as root / glue.path
     */
    void mergeGlue(const MapConfig &tilesetMapConfig
                   , const Glue &glue
                   , const boost::filesystem::path &root);
};

/** Save map config into stream.
 */
void saveMapConfig(const MapConfig &mapConfig, std::ostream &os);

} } // namespace vadstena::vts

#endif // vadstena_libs_vts_mapconfig_hpp_included_
