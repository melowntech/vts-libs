#include <iterator>

#include <boost/uuid/string_generator.hpp>
#include <boost/uuid/uuid_io.hpp>

#include "dbglog/dbglog.hpp"
#include "jsoncpp/as.hpp"

#include "./config.hpp"
#include "./detail.hpp"
#include "./driver.hpp"
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

template <typename OutputIterator>
void parseIdArray(OutputIterator out, const Json::Value &object
                  , const char *name)
{
    const Json::Value &value(object[name]);

    if (!value.isArray()) {
        LOGTHROW(err1, Json::Error)
            << "Type of " << name << " is not a list.";
    }

    for (const auto &element : value) {
        Json::check(element, Json::stringValue);
        *out++ = element.asString();
    }
}

template <typename InputIterator>
Json::Value buildIdArray(InputIterator b, InputIterator e)
{
    Json::Value value(Json::arrayValue);
    for (; b != e; ++b) {
        value.append(*b);
    }
    return value;
}

boost::any parsePlainDriver(const Json::Value &value)
{
    driver::PlainOptions driverOptions;

    int binaryOrder;
    Json::get(binaryOrder, value, "binaryOrder");
    driverOptions.binaryOrder(binaryOrder);

    std::string uuid;
    Json::get(uuid, value, "uuid");
    driverOptions.uuid(boost::uuids::string_generator()(uuid));

    return driverOptions;
}

boost::any parseAggregatedDriver(const Json::Value &value)
{
    driver::AggregatedOptions driverOptions;

    std::string storagePath;
    Json::get(storagePath, value, "storage");
    driverOptions.storagePath = storagePath;

    parseIdArray(std::inserter(driverOptions.tilesets
                               , driverOptions.tilesets.begin())
                 , value, "tilesets");

    return driverOptions;
}

boost::any parseRemoteDriver(const Json::Value &value)
{
    driver::RemoteOptions driverOptions;

    Json::get(driverOptions.url, value, "url");

    return driverOptions;
}

boost::any parseLocalDriver(const Json::Value &value)
{
    driver::LocalOptions driverOptions;

    std::string str;
    Json::get(str, value, "path");
    driverOptions.path = str;

    return driverOptions;
}

boost::any parseDriver(const Json::Value &value)
{
    // support for driver-less tileset (using by remote definition)
    if (value.isNull()) { return {}; }

    if (!value.isObject()) {
        LOGTHROW(err1, Json::Error)
            << "Type of driver is not an object.";
    }

    auto type([&]() -> std::string
    {
        if (!value.isMember("type")) { return "plain"; }

        std::string type;
        Json::get(type, value, "type");
        return type;
    }());

    if (type == "plain") {
        return parsePlainDriver(value);
    } else if (type == "aggregated") {
        return parseAggregatedDriver(value);
    } else if (type == "remote") {
        return parseRemoteDriver(value);
    } else if (type == "local") {
        return parseLocalDriver(value);
    }

    LOGTHROW(err1, Json::Error)
        << "Unrecognized driver options.";
    throw;
}

Json::Value buildDriver(const boost::any &d)
{
    Json::Value value(Json::objectValue);

    if (auto opts = boost::any_cast<const driver::PlainOptions>(&d)) {
        value["type"] = "plain";
        value["binaryOrder"] = opts->binaryOrder();
        value["uuid"] = to_string(opts->uuid());
        return value;
    } else if (auto opts = boost::any_cast
               <const driver::AggregatedOptions>(&d))
    {
        value["type"] = "aggregated";
        value["storage"] = opts->storagePath.string();
        value["tilesets"] = buildIdArray(opts->tilesets.begin()
                                         , opts->tilesets.end());
        return value;
    } else if (auto opts = boost::any_cast
               <const driver::RemoteOptions>(&d))
    {
        value["type"] = "remote";
        value["url"] = opts->url;
        return value;
    } else if (auto opts = boost::any_cast
               <const driver::LocalOptions>(&d))
    {
        value["type"] = "local";
        value["path"] = opts->path.string();
        return value;
    }

    LOGTHROW(err1, Json::Error)
        << "Dunno how to serialize driver options.";
    throw;
}

FullTileSetProperties parse1(const Json::Value &config)
{
    FullTileSetProperties properties;
    Json::get(properties.id, config, "id");

    Json::get(properties.referenceFrame, config, "referenceFrame");
    Json::get(properties.revision, config, "revision");

    parseIdSet(properties.credits, config, "credits");
    parseIdSet(properties.boundLayers, config, "boundLayers");

    properties.position = registry::positionFromJson(config["position"]);

    // load driver options
    properties.driverOptions = parseDriver(config["driver"]);

    Json::get(properties.lodRange.min, config, "lodRange", 0);
    Json::get(properties.lodRange.max, config, "lodRange", 1);

    Json::get(properties.tileRange.ll(0), config, "tileRange", 0);
    Json::get(properties.tileRange.ll(1), config, "tileRange", 1);
    Json::get(properties.tileRange.ur(0), config, "tileRange", 2);
    Json::get(properties.tileRange.ur(1), config, "tileRange", 3);

    parseSpatialDivisionExtents(properties.spatialDivisionExtents
                                , config, "spatialDivisionExtents");

    if (config.isMember("mergeBottomLod")) {
        Json::get(properties.mergeBottomLod, config, "mergeBottomLod");
    } else {
        properties.mergeBottomLod = 0;
    }

    return properties;
}

void build(Json::Value &config, const FullTileSetProperties &properties)
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

    if (!properties.driverOptions.empty()) {
        config["driver"] = buildDriver(properties.driverOptions);
    }

    auto &lodRange(config["lodRange"] = Json::arrayValue);
    lodRange.append(properties.lodRange.min);
    lodRange.append(properties.lodRange.max);

    auto &tileRange(config["tileRange"] = Json::arrayValue);
    tileRange.append(properties.tileRange.ll(0));
    tileRange.append(properties.tileRange.ll(1));
    tileRange.append(properties.tileRange.ur(0));
    tileRange.append(properties.tileRange.ur(1));

    build(config["spatialDivisionExtents"], properties.spatialDivisionExtents);

    if (properties.mergeBottomLod) {
        config["mergeBottomLod"] = properties.mergeBottomLod;
    }
}

} // namespace detail

FullTileSetProperties loadConfig(std::istream &in
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

void saveConfig(std::ostream &out, const FullTileSetProperties &properties)
{
    Json::Value config;
    detail::build(config, properties);
    out.precision(15);
    Json::StyledStreamWriter().write(out, config);
}

FullTileSetProperties loadConfig(const boost::filesystem::path &path)
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
                , const FullTileSetProperties &properties)
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

    if (config.isMember("freeLayers")) {
        ep.freeLayers = registry::freeLayersFromJson(config["freeLayers"]);
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

    // browser config options -- whole JSON object held in opaque pointer.
    if (config.isMember("browserOptions")) {
        const auto &bco(config["browserOptions"]);
        Json::check(bco, Json::objectValue);
        ep.browserOptions = bco;
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

boost::optional<unsigned int> loadRevision(std::istream &in)
{
    // load json
    Json::Value config;
    Json::Reader reader;
    if (!reader.parse(in, config)) { return boost::none; }

    auto r(config["revision"]);
    if (!r.isUInt()) { return boost::none; }
    return r.asUInt();
}

boost::optional<unsigned int> loadRevision(const boost::filesystem::path &path)
{
    LOG(info1) << "Loading extra revision from " << path  << ".";
    std::ifstream f;
    f.exceptions(std::ios::badbit | std::ios::failbit);
    try {
        f.open(path.string(), std::ios_base::in);
        auto r(loadRevision(f));
        f.close();
        return r;
    } catch (const std::exception &) {}
    return boost::none;
}

FullTileSetProperties loadConfig(const Driver &driver)
{
    try {
        // load config
        auto f(driver.input(File::config));
        const auto p(tileset::loadConfig(*f));
        f->close();

        // set
        return p;
    } catch (const std::exception &e) {
        LOGTHROW(err1, storage::Error)
            << "Unable to read config: <" << e.what() << ">.";
    }
    throw;
}

FullTileSetProperties loadConfig(const IStream::pointer &file)
{
    try {
        // load config
        const auto p(tileset::loadConfig(*file));
        file->close();

        // set
        return p;
    } catch (const std::exception &e) {
        LOGTHROW(err1, storage::Error)
            << "Unable to read config: <" << e.what() << ">.";
    }
    throw;
}

} } } // namespace vadstena::vts::tileset
