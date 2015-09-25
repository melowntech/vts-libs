#include "../registry/json.hpp"

#include "./tileop.hpp"
#include "./mapconfig.hpp"

namespace vadstena { namespace vts {

namespace {
    const int VERSION = 1;
} // namespace

const char* MapConfig::contentType("application/json");

Json::Value asJson(const Surface &surface
                   , registry::BoundLayer::dict &boundLayers)
{
    Json::Value s(Json::objectValue);

    s["id"] = surface.id;

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

    return s;
}

Json::Value asJson(const Surface::list &surfaces
                   , registry::BoundLayer::dict &boundLayers)
{
    Json::Value s(Json::arrayValue);
    for (const auto &surface : surfaces) {
        s.append(asJson(surface, boundLayers));
    }
    return s;
}

Json::Value asJson(const View &view
                   , registry::BoundLayer::dict &boundLayers)
{
    Json::Value v(Json::objectValue);

    auto &surfaces(v["surfaces"] = Json::arrayValue);
    for (const auto &surface : view.surfaces) {
        surfaces.append(surface);
    }

    auto &bls(v["boundLayers"] = Json::arrayValue);
    for (const auto &bl : view.boundLayers) {
        bls.append(bl);
        boundLayers.add(registry::Registry::boundLayer(bl));
    }

    v["freeLayers"] = Json::arrayValue;

    return v;
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

    content["position"] = registry::asJson(mapConfig.position);

    // not implemented (so far)
    content["freeLayers"] = Json::objectValue;
    content["glue"] = Json::arrayValue;
    content["rois"] = Json::arrayValue;
    content["view"] = asJson(mapConfig.view, boundLayers);
    content["namedViews"] = Json::arrayValue;

    // dunno what to put here...
    content["params"] = Json::objectValue;

    for (const auto &bl : boundLayers) {
        credits.update(registry::creditsAsDict(bl.second.credits));
    }

    content["credits"] = registry::asJson(credits);
    content["boundLayers"] = registry::asJson(boundLayers);

    content["textureAtlasReady"] = mapConfig.textureAtlasReady;

    os.precision(15);
    Json::StyledStreamWriter().write(os, content);
}

} } // namespace vadstena::vts
