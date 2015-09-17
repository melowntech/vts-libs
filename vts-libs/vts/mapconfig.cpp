#include "../registry/json.hpp"

#include "./mapconfig.hpp"

namespace vadstena { namespace vts {

void saveMapConfig(const MapConfig &mapConfig, std::ostream &os)
{
    Json::Value content;
    content["srses"] = registry::asJson(mapConfig.srs);
    content["referenceFrame"] = registry::asJson(mapConfig.referenceFrame);
    content["credits"] = registry::asJson(mapConfig.credits);
    content["boundLayers"] = registry::asJson(mapConfig.boundLayers);

    os.precision(15);
    Json::StyledStreamWriter().write(os, content);
}

} } // namespace vadstena::vts
