/**
 * \file vts/mapconfig.hpp
 * \author Vaclav Blazek <vaclav.blazek@citationtech.net>
 */

#ifndef vadstena_libs_vts_mapconfig_hpp_included_
#define vadstena_libs_vts_mapconfig_hpp_included_

#include <memory>
#include <iostream>

#include <boost/filesystem/path.hpp>

#include "../registry.hpp"
#include "./basetypes.hpp"
#include "./glue.hpp"

namespace vadstena { namespace vts {

struct SurfaceUrls3d {
    std::string meta;
    std::string mesh;
    std::string texture;
    std::string nav;
};

struct SurfaceUrls2d {
    std::string meta;
    std::string mask;
    std::string ortho;
    std::string credits;
};

struct SurfaceCommonConfig {
    /** Root makes sense only when generating mapconfig.
     */
    boost::filesystem::path root;
    storage::LodRange lodRange;
    registry::TileRange tileRange;
    boost::optional<std::string> textureLayer;
    bool has2dInterface;
    unsigned int revision;

    /** If not present values are auto filled from root and default templates.
     */
    boost::optional<SurfaceUrls3d> urls3d;

    /** If not present values are auto filled from root and default templates.
     *  Only when has2dInterface is true.
     */
    boost::optional<SurfaceUrls2d> urls2d;

    SurfaceCommonConfig() : has2dInterface(false), revision(0) {}
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

/** Opaque to outer world.
 */
struct BrowserOptions;

/** Full map configuration.
 *  Inherited from Registry as it can be used in a registry context.
 */
struct MapConfig : public registry::Registry {
    registry::ReferenceFrame referenceFrame;
    registry::Position position;
    bool textureAtlasReady;

    SurfaceConfig::list surfaces;
    GlueConfig::list glues;

    registry::Roi::list rois;
    registry::View view;
    registry::View::map namedViews;

    /** Opaque structure holding browser setup.
     */
    std::shared_ptr<BrowserOptions> browserOptions;

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

    /** Merges in other map config.
     *
     *  Copies reference frame from other configuration if not set yet;
     *  otherwise check for same reference frame (only ID, not content).
     */
    void merge(const MapConfig &other);
};

/** Configuration for meshTiles free layer.
 */
struct MeshTilesConfig {
    SurfaceConfig surface;
    registry::Credit::dict credits;

    static const char *contentType;
};

/** Save map config into stream.
 */
void saveMapConfig(const MapConfig &mapConfig, std::ostream &os);

/** Load map config from a stream.
 */
void loadMapConfig(MapConfig &mapConfig, std::istream &is
                   , const boost::filesystem::path &path = "unknown");

/** Convert meshTiles config into free layer definition.
 */
registry::FreeLayer freeLayer(const MeshTilesConfig &config);

} } // namespace vadstena::vts

#endif // vadstena_libs_vts_mapconfig_hpp_included_
