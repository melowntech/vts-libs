#include "../registry/json.hpp"

#include "./tileop.hpp"
#include "./mapconfig.hpp"

namespace vadstena { namespace vts {

Json::Value asJson(const Surface &surface)
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
        = (surface.root / fileTemplate(storage::TileFile::meta)).string();
    s["meshUrl"]
        = (surface.root / fileTemplate(storage::TileFile::mesh)).string();
    s["textureUrl"]
        = (surface.root / fileTemplate(storage::TileFile::atlas)).string();
    s["navUrl"]
        = (surface.root / fileTemplate(storage::TileFile::navtile)).string();

    return s;
}

Json::Value asJson(const Surface::list &surfaces)
{
    Json::Value s(Json::arrayValue);
    for (const auto &surface : surfaces) {
        s.append(asJson(surface));
    }
    return s;
}

void saveMapConfig(const MapConfig &mapConfig, std::ostream &os)
{
    Json::Value content;
    content["srses"] = registry::asJson(mapConfig.srs);
    content["referenceFrame"] = registry::asJson(mapConfig.referenceFrame);
    content["credits"] = registry::asJson(mapConfig.credits);
    content["boundLayers"] = registry::asJson(mapConfig.boundLayers);

    content["surfaces"] = asJson(mapConfig.surfaces);

    os.precision(15);
    Json::StyledStreamWriter().write(os, content);
}

} } // namespace vadstena::vts
