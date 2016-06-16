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

    if (surface.textureLayer) {
        s["textureLayer"] = *surface.textureLayer;
        boundLayers.add
            (registry::Registry::boundLayer(*surface.textureLayer));
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

void saveMapConfig(const MapConfig &mapConfig, std::ostream &os)
{
    Json::Value content;
    content["version"] = VERSION;

    content["srses"] = registry::asJson(mapConfig.srs);
    content["referenceFrame"] = registry::asJson(mapConfig.referenceFrame);

    auto boundLayers(mapConfig.boundLayers);

    // get credits, append all from bound layers
    auto credits(mapConfig.credits);

    content["surfaces"] = asJson(mapConfig.surfaces, boundLayers);
    content["glue"] = asJson(mapConfig.glues, boundLayers);

    content["position"] = registry::asJson(mapConfig.position);

    // not implemented (so far)
    content["freeLayers"] = Json::objectValue;
    content["rois"] = registry::asJson(mapConfig.rois);
    content["view"] = registry::asJson(mapConfig.view, boundLayers);
    content["namedViews"]
        = registry::asJson(mapConfig.namedViews, boundLayers);;

    // dunno what to put here...
    content["params"] = Json::objectValue;

    for (const auto &bl : boundLayers) {
        credits.update(registry::creditsAsDict(bl.second.credits));
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

} } // namespace vadstena::vts
