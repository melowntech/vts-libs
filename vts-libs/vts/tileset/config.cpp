#include <boost/uuid/string_generator.hpp>
#include <boost/uuid/uuid_io.hpp>

#include "dbglog/dbglog.hpp"
#include "jsoncpp/as.hpp"

#include "./config.hpp"
#include "./detail.hpp"
#include "../../storage/error.hpp"
#include "../../registry/json.hpp"

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

void parseIdSet(registry::IdSet &ids, const Json::Value &object
                , const char *name)
{
    const Json::Value &value(object[name]);

    if (!value.isArray()) {
        LOGTHROW(err1, Json::Error)
            << "Type of " << name << " is not a list.";
    }

    for (const auto &element : value) {
        Json::check(element, Json::intValue);
        ids.insert(element.asInt());
    }
}

void parseIdSet(registry::StringIdSet &ids, const Json::Value &object
                , const char *name)
{
    const Json::Value &value(object[name]);

    if (!value.isArray()) {
        LOGTHROW(err1, Json::Error)
            << "Type of " << name << " is not a list.";
    }

    for (const auto &element : value) {
        Json::check(element, Json::stringValue);
        ids.insert(element.asString());
    }
}

TileSet::Properties parse1(const Json::Value &config)
{
    TileSet::Properties properties;
    Json::get(properties.id, config, "id");

    Json::get(properties.referenceFrame, config, "referenceFrame");
    Json::get(properties.revision, config, "revision");

    parseIdSet(properties.credits, config, "credits");
    parseIdSet(properties.boundLayers, config, "boundLayers");

    parse(properties.position, config["position"]);

    // load driver options
    const auto &driver(config["driver"]);

    int binaryOrder;
    Json::get(binaryOrder, driver, "binaryOrder");
    properties.driverOptions.binaryOrder(binaryOrder);

    std::string uuid;
    Json::get(uuid, driver, "uuid");
    properties.driverOptions.uuid(boost::uuids::string_generator()(uuid));

    Json::get(properties.lodRange.min, config, "lodRange", 0);
    Json::get(properties.lodRange.max, config, "lodRange", 1);

    Json::get(properties.tileRange.ll(0), config, "tileRange", 0);
    Json::get(properties.tileRange.ll(1), config, "tileRange", 1);
    Json::get(properties.tileRange.ur(0), config, "tileRange", 2);
    Json::get(properties.tileRange.ur(1), config, "tileRange", 3);

    return properties;
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
    auto &boundLayers(config["boundLayers"] = Json::arrayValue);
    for (auto cid : properties.boundLayers) { boundLayers.append(cid); }

    config["position"] = registry::asJson(properties.position);

    auto &driver(config["driver"]);
    driver["binaryOrder"] = properties.driverOptions.binaryOrder();
    driver["uuid"] = to_string(properties.driverOptions.uuid());

    auto &lodRange(config["lodRange"] = Json::arrayValue);
    lodRange.append(properties.lodRange.min);
    lodRange.append(properties.lodRange.max);

    auto &tileRange(config["tileRange"] = Json::arrayValue);
    tileRange.append(properties.tileRange.ll(0));
    tileRange.append(properties.tileRange.ll(1));
    tileRange.append(properties.tileRange.ur(0));
    tileRange.append(properties.tileRange.ur(1));
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

namespace detail_extra {

ExtraProperties parse1(const Json::Value &config)
{
    ExtraProperties ep;

    if (config.isMember("position")) {
        ep.position = boost::in_place();
        detail::parse(*ep.position, config["position"]);
    }

    if (config.isMember("textureLayer")) {
        ep.textureLayer = boost::in_place();
        Json::get(*ep.textureLayer, config, "textureLayer");
    }

    if (config.isMember("extraCredits")) {
        detail::parseIdSet(ep.extraCredits, config, "extraCredits");
    }

    if (config.isMember("extraBoundLayers")) {
        detail::parseIdSet(ep.extraBoundLayers, config, "extraBoundLayers");
    }

    return ep;
}

} // namespace detail_extra

ExtraProperties loadExtraConfig(std::istream &in)
{
    // load json
    Json::Value config;
    Json::Reader reader;
    if (!reader.parse(in, config)) {
        LOGTHROW(err2, storage::FormatError)
            << "Unable to parse extra config: "
            << reader.getFormattedErrorMessages() << ".";
    }

    try {
        int version(0);
        Json::get(version, config, "version");

        switch (version) {
        case 1:
            return detail_extra::parse1(config);
        }

        LOGTHROW(err1, storage::FormatError)
            << "Invalid extra config format: unsupported version "
            << version << ".";

    } catch (const Json::Error &e) {
        LOGTHROW(err1, storage::FormatError)
            << "Invalid extra config format (" << e.what()
            << "); Unable to work with this config.";
    }
    throw;
}

ExtraProperties loadExtraConfig(const boost::filesystem::path &path)
{
    LOG(info1) << "Loading extra config from " << path  << ".";
    std::ifstream f;
    f.exceptions(std::ios::badbit | std::ios::failbit);
    try {
        f.open(path.string(), std::ios_base::in);
    } catch (const std::exception &e) {
        LOGTHROW(err1, storage::NoSuchTileSet)
            << "Unable to load extra config file " << path << ".";
    }
    auto p(loadExtraConfig(f));
    f.close();
    return p;
}

} } // namespace vadstena::vts
