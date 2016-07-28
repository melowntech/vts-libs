#include "dbglog/dbglog.hpp"

#include "jsoncpp/as.hpp"

#include "../registry/json.hpp"
#include "../storage/error.hpp"

#include "./json.hpp"
#include "./tileop.hpp"
#include "./mapconfig.hpp"

namespace vadstena { namespace vts {

namespace {
    const int VERSION = 1;
} // namespace

const char* MapConfig::contentType("application/json; charset=utf-8");
const char* MeshTilesConfig::contentType("application/json; charset=utf-8");

Json::Value asJson(const Glue::Id &id)
{
    Json::Value value(Json::arrayValue);
    for (const auto &str : id) { value.append(str); }
    return value;
}

void asJson(const SurfaceCommonConfig &surface, Json::Value &s
            , registry::BoundLayer::dict &boundLayers)
{
    auto &lodRange(s["lodRange"] = Json::arrayValue);
    lodRange.append(surface.lodRange.min);
    lodRange.append(surface.lodRange.max);

    auto &tileRange(s["tileRange"] = Json::arrayValue);
    auto &tileRangeMin(tileRange.append(Json::arrayValue));
    tileRangeMin.append(surface.tileRange.ll(0));
    tileRangeMin.append(surface.tileRange.ll(1));
    auto &tileRangeMax(tileRange.append(Json::arrayValue));
    tileRangeMax.append(surface.tileRange.ur(0));
    tileRangeMax.append(surface.tileRange.ur(1));

    // paths
    if (surface.urls3d) {
        s["metaUrl"] = surface.urls3d->meta;
        s["meshUrl"] = surface.urls3d->mesh;
        s["textureUrl"] = surface.urls3d->texture;
        s["navUrl"] = surface.urls3d->nav;
    } else {
        s["metaUrl"]
            = (surface.root / fileTemplate(storage::TileFile::meta
                                           , surface.revision)).string();
        s["meshUrl"]
            = (surface.root / fileTemplate(storage::TileFile::mesh
                                           , surface.revision)).string();
        s["textureUrl"]
            = (surface.root / fileTemplate(storage::TileFile::atlas
                                           , surface.revision)).string();
        s["navUrl"]
            = (surface.root / fileTemplate(storage::TileFile::navtile
                                           , surface.revision)).string();
    }

    if (surface.has2dInterface) {
        auto &i2d(s["2d"] = Json::objectValue);
        if (surface.urls2d) {
            s["metaUrl"] = surface.urls2d->meta;
            s["maskUrl"] = surface.urls2d->mask;
            s["orthoUrl"] = surface.urls2d->ortho;
            s["creditsUrl"] = surface.urls2d->credits;
        } else {
            i2d["metaUrl"]
                = (surface.root / fileTemplate(storage::TileFile::meta2d
                                               , surface.revision)).string();
            i2d["maskUrl"]
                = (surface.root / fileTemplate(storage::TileFile::mask
                                               , surface.revision)).string();
            i2d["orthoUrl"]
                = (surface.root / fileTemplate(storage::TileFile::ortho
                                               , surface.revision)).string();
            i2d["creditsUrl"]
                = (surface.root / fileTemplate(storage::TileFile::credits
                                               , surface.revision)).string();
        }
    }

    if (surface.textureLayer) {
        s["textureLayer"] = *surface.textureLayer;
        boundLayers.add
            (registry::system.boundLayers(*surface.textureLayer));
    }
}

Json::Value asJson(const SurfaceConfig &surface
                   , registry::BoundLayer::dict &boundLayers)
{
    Json::Value s(Json::objectValue);
    s["id"] = surface.id;
    asJson(surface, s, boundLayers);
    return s;
}

Json::Value asJson(const GlueConfig &glue
                   , registry::BoundLayer::dict &boundLayers)
{
    Json::Value s(Json::objectValue);
    s["id"] = asJson(glue.id);
    asJson(glue, s, boundLayers);
    return s;
}

Json::Value asJson(const SurfaceConfig::list &surfaces
                   , registry::BoundLayer::dict &boundLayers)
{
    Json::Value s(Json::arrayValue);
    for (const auto &surface : surfaces) {
        s.append(asJson(surface, boundLayers));
    }
    return s;
}

Json::Value asJson(const GlueConfig::list &glues
                   , registry::BoundLayer::dict &boundLayers)
{
    Json::Value s(Json::arrayValue);
    for (const auto &glue : glues) {
        s.append(asJson(glue, boundLayers));
    }
    return s;
}

// parsing

void fromJson(Glue::Id &id, const Json::Value &value)
{
    for (const auto &element : value) {
        Json::check(element, Json::stringValue);
        id.push_back(element.asString());
    }
}

void fromJson(SurfaceCommonConfig &surface, const Json::Value &value)
{
    Json::get(surface.lodRange.min, value, "lodRange", 0);
    Json::get(surface.lodRange.max, value, "lodRange", 1);

    surface.tileRange = registry::tileRangeFromJson(value["tileRange"]);

    surface.urls3d = boost::in_place();
    Json::get(surface.urls3d->meta, value, "metaUrl");
    Json::get(surface.urls3d->mesh, value, "meshUrl");
    Json::get(surface.urls3d->texture, value, "textureUrl");
    Json::get(surface.urls3d->nav, value, "navUrl");

    if (value.isMember("2d")) {
        surface.has2dInterface = true;
        const auto &i2d(value["2d"]);
        surface.urls2d = boost::in_place();
        Json::get(surface.urls2d->meta, i2d, "metaUrl");
        Json::get(surface.urls2d->mask, i2d, "maskUrl");
        Json::get(surface.urls2d->ortho, i2d, "orthoUrl");
        Json::get(surface.urls2d->credits, i2d, "creditsUrl");
    }

    if (value.isMember("textureLayer")) {
        surface.textureLayer = boost::in_place();
        Json::get(*surface.textureLayer, value, "textureLayer");
    }
}

void fromJson(SurfaceConfig &surface, const Json::Value &value)
{
    Json::get(surface.id, value, "id");
    fromJson(static_cast<SurfaceCommonConfig&>(surface), value);
}

void fromJson(GlueConfig &glue, const Json::Value &value)
{
    fromJson(glue.id, value["id"]);
    fromJson(static_cast<SurfaceCommonConfig&>(glue), value);
}

void fromJson(SurfaceConfig::list &surfaces
              , const Json::Value &value)
{
    for (const auto &surface : value) {
        surfaces.emplace_back();
        fromJson(surfaces.back(), surface);
    }
}

void fromJson(GlueConfig::list &surfaces
              , const Json::Value &value)
{
    for (const auto &surface : value) {
        surfaces.emplace_back();
        fromJson(surfaces.back(), surface);
    }
}

void mergeRest(MapConfig &out, const MapConfig &in, bool surface)
{
    out.srs.update(in.srs);
    out.credits.update(in.credits);
    out.boundLayers.update(in.boundLayers);

    // merge views
    if (surface) {
        out.view.merge(in.view);

        // TODO: find out first valid
        out.position = in.position;
    }

    // all inputs must be texture-atlas-ready
    out.textureAtlasReady &= in.textureAtlasReady;
}

void MapConfig::mergeTileSet(const MapConfig &tilesetMapConfig
                             , const boost::filesystem::path &root)
{
    if (tilesetMapConfig.surfaces.size() != 1) {
        LOGTHROW(err1, storage::NoSuchTileSet)
            << "Cannot merge tileset mapConfig: "
            "there must be just one surface in the input mapConfig.";
    }

    SurfaceConfig s(tilesetMapConfig.surfaces.front());
    // set root if not set so far
    if (s.root.empty()) { s.root = root; }
    surfaces.push_back(s);

    mergeRest(*this, tilesetMapConfig, true);
}

void MapConfig::mergeGlue(const MapConfig &tilesetMapConfig
                          , const Glue &glue
                          , const boost::filesystem::path &root)
{
    if (tilesetMapConfig.surfaces.size() != 1) {
        LOGTHROW(err1, storage::NoSuchTileSet)
            << "Cannot merge tileset mapConfig as a glue: "
            "there must be just one surface in the input mapConfig.";
    }

    GlueConfig g(tilesetMapConfig.surfaces.front());
    g.id = glue.id;
    g.root = root / glue.path;
    glues.push_back(g);

    mergeRest(*this, tilesetMapConfig, false);
}

void MapConfig::merge(const MapConfig &other)
{
    if (referenceFrame.id.empty()) {
        // assign reference frame
        if (other.referenceFrame.id.empty()) {
            LOGTHROW(err1, storage::InconsistentInput)
                << "Missing referenceFrame while merging "
                "map configuration.";
        }
        referenceFrame = other.referenceFrame;
    } else if (referenceFrame.id != other.referenceFrame.id) {
        // mismatch
        LOGTHROW(err1, storage::InconsistentInput)
            << "Cannot merge map configuration for reference frame <"
            << referenceFrame.id
            << "> with map configuration for reference frame <"
            << other.referenceFrame.id << ">.";
    }

    surfaces.insert(surfaces.end(), other.surfaces.begin()
                    , other.surfaces.end());
    glues.insert(glues.end(), other.glues.begin(), other.glues.end());
    mergeRest(*this, other, true);
}

void saveMapConfig(const MapConfig &mapConfig, std::ostream &os)
{
    Json::Value content;
    content["version"] = VERSION;

    content["srses"] = registry::asJson(mapConfig.srs, true);
    content["referenceFrame"] = registry::asJson(mapConfig.referenceFrame);

    auto boundLayers(mapConfig.boundLayers);

    // get credits, append all from bound layers
    auto credits(mapConfig.credits);

    content["surfaces"] = asJson(mapConfig.surfaces, boundLayers);
    content["glue"] = asJson(mapConfig.glues, boundLayers);

    content["position"] = registry::asJson(mapConfig.position);

    content["freeLayers"] = registry::asJson(mapConfig.freeLayers);
    content["rois"] = registry::asJson(mapConfig.rois);
    content["view"] = registry::asJson(mapConfig.view, boundLayers);
    content["namedViews"]
        = registry::asJson(mapConfig.namedViews, boundLayers);;

    // dunno what to put here...
    content["params"] = Json::objectValue;

    for (const auto &bl : boundLayers) {
        credits.update(registry::creditsAsDict(bl.credits));
    }

    // grab credits
    content["credits"] = registry::asJson(credits);
    // get bound layers, no inline credits
    content["boundLayers"] = registry::asJson(boundLayers, false);

    content["textureAtlasReady"] = mapConfig.textureAtlasReady;

    // add browser core options if present
    if (mapConfig.browserOptions) {
        content["browserOptions"] = mapConfig.browserOptions->value;
    }

    os.precision(15);
    Json::StyledStreamWriter().write(os, content);
}

namespace detail {

void parse1(MapConfig &mapConfig, const Json::Value &config)
{
    fromJson(mapConfig.srs, config["srses"]);
    fromJson(mapConfig.referenceFrame, config["referenceFrame"]);
    fromJson(mapConfig.credits, config["credits"]);
    fromJson(mapConfig.boundLayers, config["boundLayers"]);

    fromJson(mapConfig.surfaces, config["surfaces"]);
    fromJson(mapConfig.glues, config["glue"]);

    fromJson(mapConfig.view, config["view"]);
    fromJson(mapConfig.namedViews, config["namedViews"]);
}

} // namespace detail

void loadMapConfig(MapConfig &mapConfig, std::istream &in
                   , const boost::filesystem::path &path)
{
    // load json
    Json::Value config;
    Json::Reader reader;
    if (!reader.parse(in, config)) {
        LOGTHROW(err2, storage::FormatError)
            << "Unable to parse map config " << path << ": "
            << reader.getFormattedErrorMessages() << ".";
    }

    try {
        int version(0);
        Json::get(version, config, "version");

        switch (version) {
        case 1:
            detail::parse1(mapConfig, config);
            return;
        }

        LOGTHROW(err1, storage::FormatError)
            << "Invalid map config format: unsupported version "
            << version << ".";

    } catch (const Json::Error &e) {
        LOGTHROW(err1, storage::FormatError)
            << "Invalid map config format (" << e.what()
            << ") in " << path << ".";
    }
    throw;
}

registry::FreeLayer freeLayer(const MeshTilesConfig &config)
{
    registry::FreeLayer fl;

    fl.id = config.surface.id;
    for (const auto &credit : config.credits) {
        fl.credits.set(credit.id, credit);
    }

    auto &def(fl.createDefinition<registry::FreeLayer::MeshTiles>());

    const auto &surface(config.surface);
    def.lodRange = surface.lodRange;
    def.tileRange = surface.tileRange;

    def.metaUrl
        = (surface.root / fileTemplate(storage::TileFile::meta
                                       , surface.revision)).string();
    def.meshUrl
        = (surface.root / fileTemplate(storage::TileFile::mesh
                                       , surface.revision)).string();

    def.textureUrl
        = (surface.root / fileTemplate(storage::TileFile::atlas
                                       , surface.revision)).string();
    // done
    return fl;
}

} } // namespace vadstena::vts
