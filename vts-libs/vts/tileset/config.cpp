#include <boost/uuid/string_generator.hpp>
#include <boost/uuid/uuid_io.hpp>

#include "dbglog/dbglog.hpp"
#include "jsoncpp/as.hpp"

#include "./config.hpp"
#include "./detail.hpp"
#include "../../storage/error.hpp"

namespace vadstena { namespace vts {

namespace detail {

const int CURRENT_JSON_FORMAT_VERSION(1);

void parse(registry::Position &p, const Json::Value &value)
{
    if (!value.isArray()) {
        LOGTHROW(err1, Json::Error)
            << "Type of position is not a list.";
    }

    p.type = boost::lexical_cast<registry::Position::Type>
        (Json::as<std::string>(value[0]));
    p.position(0) = Json::as<double>(value[1]);
    p.position(1) = Json::as<double>(value[2]);
    p.position(2) = Json::as<double>(value[3]);

    p.orientation(0) = Json::as<double>(value[4]);
    p.orientation(1) = Json::as<double>(value[5]);
    p.orientation(2) = Json::as<double>(value[6]);

    p.viewHeight = Json::as<double>(value[7]);
    p.verticalFov = Json::as<double>(value[8]);
}

TileSet::Properties parse1(const Json::Value &config)
{
    TileSet::Properties properties;
    Json::get(properties.id, config, "id");

    Json::get(properties.referenceFrame, config, "referenceFrame");
    Json::get(properties.revision, config, "revision");

    const auto &credits(config["credits"]);
    if (!credits.isArray()) {
        LOGTHROW(err1, Json::Error)
            << "Type of credits is not a list.";
    }

    for (const auto &element : credits) {
        Json::check(element, Json::stringValue);
        properties.credits.insert(element.asString());
    }

    parse(properties.position, config["position"]);


    // load driver options
    const auto &driver(config["driver"]);

    int binaryOrder;
    Json::get(binaryOrder, driver, "binaryOrder");
    properties.driverOptions.binaryOrder(binaryOrder);

    std::string uuid;
    Json::get(uuid, driver, "uuid");
    properties.driverOptions.uuid(boost::uuids::string_generator()(uuid));

    return properties;
}

void build(Json::Value &value, const registry::Position &p)
{
    value = Json::arrayValue;
    value.append(boost::lexical_cast<std::string>(p.type));
    value.append(p.position(0));
    value.append(p.position(1));
    value.append(p.position(2));
    value.append(p.orientation(0));
    value.append(p.orientation(1));
    value.append(p.orientation(2));
    value.append(p.viewHeight);
    value.append(p.verticalFov);
}

void build(Json::Value &config, const TileSet::Properties &properties)
{
    config["version"]
        = Json::Int64(detail::CURRENT_JSON_FORMAT_VERSION);

    config["id"] = properties.id;
    config["referenceFrame"] = properties.referenceFrame;
    config["revision"] = properties.revision;

    auto &credits(config["credits"] = Json::arrayValue);
    for (auto cid : properties.credits) { credits.append(cid); }

    build(config["position"], properties.position);

    auto &driver(config["driver"]);
    driver["binaryOrder"] = properties.driverOptions.binaryOrder();
    driver["uuid"] = to_string(properties.driverOptions.uuid());
}

} // namespace detail

TileSet::Properties loadConfig(std::istream &in)
{
    // load json
    Json::Value config;
    Json::Reader reader;
    if (!reader.parse(in, config)) {
        LOGTHROW(err2, storage::FormatError)
            << "Unable to parse config: "
            << reader.getFormattedErrorMessages() << ".";
    }

    try {
        int version(0);
        Json::get(version, config, "version");

        switch (version) {
        case 1:
            return detail::parse1(config);
        }

        LOGTHROW(err1, storage::FormatError)
            << "Invalid tileset config format: unsupported version "
            << version << ".";

    } catch (const Json::Error &e) {
        LOGTHROW(err1, storage::FormatError)
            << "Invalid tileset config format (" << e.what()
            << "); Unable to work with this tileset.";
    }
    throw;
}

void saveConfig(std::ostream &out, const TileSet::Properties &properties)
{
    Json::Value config;
    detail::build(config, properties);
    out.precision(15);
    Json::StyledStreamWriter().write(out, config);
}

TileSet::Properties loadConfig(const boost::filesystem::path &path)
{
    LOG(info1) << "Loading config from " << path  << ".";
    std::ifstream f;
    f.exceptions(std::ios::badbit | std::ios::failbit);
    try {
        f.open(path.string(), std::ios_base::in);
    } catch (const std::exception &e) {
        LOGTHROW(err1, storage::NoSuchTileSet)
            << "Unable to load config file " << path << ".";
    }
    auto p(loadConfig(f));
    f.close();
    return p;
}

void saveConfig(const boost::filesystem::path &path
                , const TileSet::Properties &properties)
{
    LOG(info1) << "Saving config to " << path  << ".";
    std::ofstream f;
    f.exceptions(std::ios::badbit | std::ios::failbit);
    f.open(path.string(), std::ios_base::out);
    saveConfig(f, properties);
    f.close();
}

} } // namespace vadstena::vts
