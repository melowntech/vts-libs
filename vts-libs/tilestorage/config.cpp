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

Properties loadConfig(const boost::filesystem::path &path)
{
    LOG(info1) << "Loading config from " << path  << ".";
    std::ifstream f;
    f.exceptions(std::ios::badbit | std::ios::failbit);
    f.open(path.string(), std::ios_base::in);
    auto p(loadConfig(f));
    f.close();
    return p;
}

void saveConfig(const boost::filesystem::path &path
                , const Properties &properties)
{
    LOG(info1) << "Saving config to " << path  << ".";
    std::ofstream f;
    f.exceptions(std::ios::badbit | std::ios::failbit);
    f.open(path.string(), std::ios_base::out);
    saveConfig(f, properties);
    f.close();
}

} } // namespace vadstena::tilestorage
