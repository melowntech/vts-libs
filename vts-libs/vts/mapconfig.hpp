/**
 * Copyright (c) 2017 Melown Technologies SE
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * *  Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
/**
 * \file vts/mapconfig.hpp
 * \author Vaclav Blazek <vaclav.blazek@citationtech.net>
 */

#ifndef vtslibs_vts_mapconfig_hpp_included_
#define vtslibs_vts_mapconfig_hpp_included_

#include <memory>
#include <functional>
#include <iostream>

#include <boost/optional.hpp>
#include <boost/variant.hpp>
#include <boost/filesystem/path.hpp>

#include "../registry.hpp"
#include "basetypes.hpp"
#include "glue.hpp"
#include "virtualsurface.hpp"

namespace vtslibs { namespace vts {

typedef boost::optional<std::string> OProxy;
typedef boost::optional<boost::filesystem::path> OPath;

/** Map configuration options.
 */
struct MapConfigOptions {
    /** Proxy name.
     */
    OProxy proxy;
};

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

/** Per-proxy surface root generator.
 *  Should return default path if proxy is not found.
 */
typedef std::function<boost::filesystem::path
                      (const OProxy&)> PerProxyRootFunction;

/** Surface root. Either given path or path generator.
 */
class SurfaceRoot {
public:
    SurfaceRoot() : root_(boost::filesystem::path()) {}
    SurfaceRoot(const boost::filesystem::path &path) : root_(path) {}
    SurfaceRoot(const PerProxyRootFunction &fn) : root_(fn) {}

    SurfaceRoot& operator=(const boost::filesystem::path &path) {
        root_ = path; return *this;
    }

    bool empty() const;

    boost::filesystem::path operator()() const {
        return operator()(boost::none);
    }

    boost::filesystem::path operator()(const OProxy &proxy) const;

    SurfaceRoot withSuffix(const boost::filesystem::path &suffix) const;

private:
    typedef boost::variant<boost::filesystem::path, PerProxyRootFunction> Root;

    SurfaceRoot(const Root &root, const OPath &suffix);

    Root root_;
    OPath suffix_;
};

struct SurfaceCommonConfig {
    /** Root makes sense only when generating mapconfig.
     */
    SurfaceRoot root;
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

struct VirtualSurfaceConfig : SurfaceCommonConfig {
    VirtualSurface::Id id;
    std::string mapping;

    VirtualSurfaceConfig() {}
    VirtualSurfaceConfig(const SurfaceConfig &surface)
        : SurfaceCommonConfig(surface)
    {}

    typedef std::vector<VirtualSurfaceConfig> list;
};

/** Configuration for meshTiles free layer.
 */
struct MeshTilesConfig {
    /** Needed only by debug, not serialized to freelayer format
     */
    std::string referenceFrame;

    SurfaceConfig surface;
    registry::Credit::dict credits;

    static const char *contentType;

    typedef std::vector<MeshTilesConfig> list;
};

/** Full map configuration.
 *  Inherited from Registry as it can be used in a registry context.
 */
struct MapConfig : public registry::Registry {
    registry::ReferenceFrame referenceFrame;
    registry::Position position;
    bool textureAtlasReady;

    SurfaceConfig::list surfaces;
    GlueConfig::list glues;
    VirtualSurfaceConfig::list virtualSurfaces;

    MeshTilesConfig::list meshTiles;

    registry::FreeLayer::dict freeLayers;

    registry::Roi::list rois;
    registry::View view;
    registry::View::map namedViews;

    /** Opaque structure holding browser setup.
     */
    boost::any browserOptions;

    registry::Service::dict services;

    static const char *contentType;

    MapConfig() : textureAtlasReady(false) {}

    MapConfig(const registry::RegistryBase &base)
        : registry::Registry(base), textureAtlasReady(false) {}

    /** Merges in mapConfig for one tileset.
     *
     * \param tilesetMapConfig single tileset mapConfig
     * \param root path to tileset
     */
    void mergeTileSet(const MapConfig &tilesetMapConfig
                      , const SurfaceRoot &root
                      , const TilesetIdMap *tilesetRename = nullptr);

    /** Merges in mapConfig for one tileset as a glue.
     *
     * \param glueMapConfig single tileset mapConfig
     * \param glue glue definition
     * \param root path to glues
     *
     * NB: glue path is composed as root / glue.path
     */
    void mergeGlue(const MapConfig &tilesetMapConfig
                   , const Glue &glue, const SurfaceRoot &root
                   , const TilesetIdMap *tilesetRename = nullptr);

    /** Merges in mapConfig for one tileset as a virtual surface.
     *
     * \param virtualSurfaceMapConfig single tileset mapConfig
     * \param virtualSurface virtualSurface definition
     * \param root path to virtualSurfaces
     *
     * NB: virtualSurface path is composed as root / virtualSurface.path
     */
    void mergeVirtualSurface(const MapConfig &tilesetMapConfig
                             , const VirtualSurface &virtualSurface
                             , const SurfaceRoot &root
                             , const TilesetIdMap *tilesetRename = nullptr);

    /** Adds in MeshTilesConfig
     *
     * \param meshTilesConfig single mesh tiles configuration
     * \param root path to tileset
     */
    void addMeshTilesConfig(const MeshTilesConfig &meshTilesConfig
                            , const SurfaceRoot &root
                            , const TilesetIdMap *tilesetRename = nullptr);

    struct MergeFlags {
        typedef std::uint32_t value_type;

        enum : value_type {
            view = 0x1
            , namedViews = 0x2
            , position = 0x4

            , none = 0x0
            , all = 0xffffffff
        }; };

    /** Merges in other map config.
     *
     *  Copies reference frame from other configuration if not set yet;
     *  otherwise check for same reference frame (only ID, not content).
     *
     *  Other stuff is merged based on the provided flags.
     */
    void merge(const MapConfig &other
               , MergeFlags::value_type flags = MergeFlags::all);
};

/** Save map config into stream.
 */
void saveMapConfig(const MapConfig &mapConfig, std::ostream &os
                   , const MapConfigOptions *mco = nullptr);

/** Load map config from a stream.
 */
void loadMapConfig(MapConfig &mapConfig, std::istream &is
                   , const boost::filesystem::path &path = "unknown");

/** Load map config from a file.
 */
MapConfig loadMapConfig(const boost::filesystem::path &path);

/** Applies given base URL to all non-absolute paths.
 */
void absolutize(MapConfig &mapConfig, const std::string &base);

/** Convert meshTiles config into free layer definition.
 */
registry::FreeLayer freeLayer(const MeshTilesConfig &config
                              , bool inlineCredits = true
                              , const boost::filesystem::path &root
                              = boost::filesystem::path()
                              , const MapConfigOptions *mco = nullptr);

/** Save map config as a list of directories into stream.
 */
void saveDirs(const MapConfig &mapConfig, std::ostream &os
              , const MapConfigOptions *mco = nullptr);

/** Debug config.
 */
struct DebugConfig {
    std::string referenceFrame;
    storage::LodRange lodRange;
    registry::TileRange tileRange;

    std::string meta;
    std::string mask;

    static const char *contentType;
};

/** Generates debug config from mesh tiles config.
 */
DebugConfig debugConfig(const MeshTilesConfig &config
                        , const boost::filesystem::path &root
                        = boost::filesystem::path());

/** Save debug config to stream.
 */
void saveDebug(std::ostream &os, const DebugConfig &debug);

} } // namespace vtslibs::vts

#endif // vtslibs_vts_mapconfig_hpp_included_
