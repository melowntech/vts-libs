#include "./config.hpp"
#include "./json.hpp"
#include "./error.hpp"

namespace vadstena { namespace tilestorage {

Properties loadConfig(std::istream &in)
{
    // load json
    Json::Value config;
    Json::Reader reader;
    if (!reader.parse(in, config)) {
        LOGTHROW(err2, FormatError)
            << "Unable to parse config: "
            << reader.getFormattedErrorMessages() << ".";
    }

    Properties p;
    parse(p, config);
    return p;
}

void saveConfig(std::ostream &out, const Properties &properties)
{
    Json::Value config;
    build(config, properties);
    out.precision(15);
    Json::StyledStreamWriter().write(out, config);
}

} } // namespace vadstena::tilestorage
