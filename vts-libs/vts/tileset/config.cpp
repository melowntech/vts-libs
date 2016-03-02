#include <boost/uuid/string_generator.hpp>
#include <boost/uuid/uuid_io.hpp>

#include "dbglog/dbglog.hpp"
#include "jsoncpp/as.hpp"

#include "./config.hpp"
#include "./detail.hpp"
#include "./driver/options.hpp"
#include "../../storage/error.hpp"
#include "../../registry/json.hpp"

namespace vadstena { namespace vts { namespace tileset {

namespace detail {

const int CURRENT_JSON_FORMAT_VERSION(1);

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
            << "Type of " << name << " is not an array.";
    }

    for (const auto &element : value) {
        Json::check(element, Json::stringValue);
        ids.insert(element.asString());
    }
}

void parseExtents(math::Extents2 &extents
                  , const Json::Value &value
                  , const char *name)
{
    if (!value.isArray() && (value.size() != 4)) {
        LOGTHROW(err1, Json::Error)
            << "Type of " << name << " is not an 4-item array.";
    }

    extents.ll(0) = Json::as<double>(value[0], name);
    extents.ll(1) = Json::as<double>(value[1], name);
    extents.ur(0) = Json::as<double>(value[2], name);
    extents.ur(1) = Json::as<double>(value[3], name);
}

void parseSpatialDivisionExtents(SpatialDivisionExtents &sde
                                 , const Json::Value &object
                                 , const char *name)
{
    const Json::Value &value(object[name]);

    if (!value.isObject()) {
        LOGTHROW(err1, Json::Error)
            << "Type of " << name << " is not an object.";
    }

    for (const auto &srs : value.getMemberNames()) {
        parseExtents(sde[srs], value[srs], srs.c_str());
    }
}

void build(Json::Value &value
           , const SpatialDivisionExtents &spatialDivisionExtents)
{
    value = Json::objectValue;
    for (const auto &item : spatialDivisionExtents) {
        auto &e(value[item.first] = Json::arrayValue);
        e.append(item.second.ll(0));
        e.append(item.second.ll(1));
        e.append(item.second.ur(0));
        e.append(item.second.ur(1));
    }
}

boost::any parsePlainDriver(const Json::Value &value)
{
    driver::PlainDriverOptions driverOptions;

    int binaryOrder;
    Json::get(binaryOrder, value, "binaryOrder");
    driverOptions.binaryOrder(binaryOrder);

    std::string uuid;
    Json::get(uuid, value, "uuid");
    driverOptions.uuid(boost::uuids::string_generator()(uuid));

    return driverOptions;
}

boost::any parseDriver(const Json::Value &value)
{
    if (!value.isObject()) {
        LOGTHROW(err1, Json::Error)
            << "Type of driver is not an object.";
    }

    if (value.isMember("binaryOrder")) {
        return parsePlainDriver(value);
    }

    LOGTHROW(err1, Json::Error)
        << "Unrecognized driver options.";
    throw;
}

Json::Value buildDriver(const boost::any &d)
{
    Json::Value value(Json::objectValue);

    if (auto opts = boost::any_cast<const driver::PlainDriverOptions>(&d)) {
        value["binaryOrder"] = opts->binaryOrder();
        value["uuid"] = to_string(opts->uuid());
        return value;
    }

    LOGTHROW(err1, Json::Error)
        << "Dunno how to serialize driver options.";
    throw;
}

TileSet::Properties parse1(const Json::Value &config)
{
    TileSet::Properties properties;
    Json::get(properties.id, config, "id");

    Json::get(properties.referenceFrame, config, "referenceFrame");
    Json::get(properties.revision, config, "revision");

    parseIdSet(properties.credits, config, "credits");
    parseIdSet(properties.boundLayers, config, "boundLayers");

    properties.position = registry::positionFromJson(config["position"]);

    // load driver options
    const auto &driver(config["driver"]);

    properties.driverOptions = parseDriver(driver);

    Json::get(properties.lodRange.min, config, "lodRange", 0);
    Json::get(properties.lodRange.max, config, "lodRange", 1);

    Json::get(properties.tileRange.ll(0), config, "tileRange", 0);
    Json::get(properties.tileRange.ll(1), config, "tileRange", 1);
    Json::get(properties.tileRange.ur(0), config, "tileRange", 2);
    Json::get(properties.tileRange.ur(1), config, "tileRange", 3);

    parseSpatialDivisionExtents(properties.spatialDivisionExtents
                                , config, "spatialDivisionExtents");

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

    config["driver"] = buildDriver(properties.driverOptions);

    auto &lodRange(config["lodRange"] = Json::arrayValue);
    lodRange.append(properties.lodRange.min);
    lodRange.append(properties.lodRange.max);

    auto &tileRange(config["tileRange"] = Json::arrayValue);
    tileRange.append(properties.tileRange.ll(0));
    tileRange.append(properties.tileRange.ll(1));
    tileRange.append(properties.tileRange.ur(0));
    tileRange.append(properties.tileRange.ur(1));

    build(config["spatialDivisionExtents"], properties.spatialDivisionExtents);
}

} // namespace detail

TileSet::Properties loadConfig(std::istream &in
                               , const boost::filesystem::path &path)
{
    // load json
    Json::Value config;
    Json::Reader reader;
    if (!reader.parse(in, config)) {
        LOGTHROW(err2, storage::FormatError)
            << "Unable to parse config " << path << ": "
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
            << "); Unable to work with this tileset (file: " << path << ").";
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
        f.peek();
    } catch (const std::exception &e) {
        LOGTHROW(err1, storage::NoSuchTileSet)
            << "Unable to load config file " << path << ".";
    }
    auto p(loadConfig(f, path));
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

ExtraTileSetProperties parse1(const Json::Value &config)
{
    ExtraTileSetProperties ep;

    if (config.isMember("position")) {
        ep.position = boost::in_place();
        *ep.position = registry::positionFromJson(config["position"]);
    }

    if (config.isMember("textureLayer")) {
        ep.textureLayer = boost::in_place();
        Json::get(*ep.textureLayer, config, "textureLayer");
    }

    if (config.isMember("credits")) {
        ep.credits = registry::creditsFromJson(config["credits"]);
    }

    if (config.isMember("boundLayers")) {
        ep.boundLayers = registry::boundLayersFromJson(config["boundLayers"]);
    }

    if (config.isMember("rois")) {
        ep.rois = registry::roisFromJson(config["rois"]);
    }

    if (config.isMember("namedViews")) {
        ep.namedViews = registry::namedViewsFromJson(config["namedViews"]);
    }

    if (config.isMember("view")) {
        ep.view = registry::viewFromJson(config["view"]);
    }

    return ep;
}

} // namespace detail_extra

ExtraTileSetProperties loadExtraConfig(std::istream &in)
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

ExtraTileSetProperties loadExtraConfig(const boost::filesystem::path &path)
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

} } } // namespace vadstena::vts::tileset
