#include <boost/lexical_cast.hpp>
#include "dbglog/dbglog.hpp"
#include "jsoncpp/as.hpp"

#include "./config.hpp"
#include "./detail.hpp"
#include "../json.hpp"
#include "../../storage/error.hpp"
#include "../../registry/json.hpp"

namespace vadstena { namespace vts { namespace storage {

namespace detail {

const int CURRENT_JSON_FORMAT_VERSION(1);

void parseList(std::vector<std::string> &ids, const Json::Value &object
               , const char *name)
{
    const Json::Value &value(object[name]);

    if (!value.isArray()) {
        LOGTHROW(err1, Json::Error)
            << "Type of " << name << " is not a list.";
    }

    for (const auto &element : value) {
        Json::check(element, Json::stringValue);
        ids.push_back(element.asString());
    }
}

void parseTileset(StoredTileset &tileset, const Json::Value &value)
{
    if (!value.isObject()) {
        LOGTHROW(err1, Json::Error)
            << "Type of tileset is not an object.";
    }

    Json::get(tileset.tilesetId, value, "id");
    if (value.isMember("version")) {
        // versioned -> get version and base ID
        Json::get(tileset.baseId, value, "base");
        Json::get(tileset.version, value, "version");
    } else {
        // non-versioned -> base sameId as tilesetId and no version
        tileset.baseId = tileset.tilesetId;
        tileset.version = 0;
    }
}

void parseTilesets(StoredTileset::list &tilesets, const Json::Value &value)
{
    if (!value.isArray()) {
        LOGTHROW(err1, Json::Error)
            << "Type of tilesets is not an array.";
    }

    for (const auto &element : value) {
        Json::check(element, Json::objectValue);
        tilesets.emplace_back();
        parseTileset(tilesets.back(), element);
    }
}

void parseGlue(Glue &glue, const Json::Value &value)
{
    if (!value.isObject()) {
        LOGTHROW(err1, Json::Error)
            << "Type of glue is not an object.";
    }

    parseList(glue.id, value, "id");
    Json::get(glue.path, value, "dir");
}

void parseGlues(Glue::map &glues, const Json::Value &value)
{
    if (!value.isArray()) {
        LOGTHROW(err1, Json::Error)
            << "Type of glues is not an array.";
    }

    for (const auto &element : value) {
        Json::check(element, Json::objectValue);
        Glue g;
        parseGlue(g, element);
        glues.insert(Glue::map::value_type(g.id, g));
    }
}

void parseTrash(TrashBin &trashBin, const Json::Value &value)
{
    // ignore missing trash bin
    if (value.isNull()) { return; }
    if (!value.isArray()) {
        LOGTHROW(err1, Json::Error)
            << "Type of trash is not an array.";
    }

    for (const auto &element : value) {
        Json::check(element, Json::objectValue);
        // Glue g;
        // parseGlue(g, element);
        // glues.insert(Glue::map::value_type(g.id, g));
    }
    (void) trashBin;
}

void buildTrash(const TrashBin &trashBin, Json::Value &object)
{
    object = Json::arrayValue;

    for (const auto &item : trashBin) {
        // buildGlue(item.second, object.append(Json::nullValue));
        (void) item;
    }
    (void) trashBin;
}

void buildList(const std::vector<std::string> &ids, Json::Value &value)
{
    value = Json::arrayValue;
    for (const auto &id : ids) { value.append(id); }
}

void buildTileset(const StoredTileset &tileset, Json::Value &object)
{
    object = Json::objectValue;
    object["id"] = tileset.tilesetId;
    if (tileset.version > 0) {
        object["base"] = tileset.baseId;
        object["version"] = tileset.version;
    }
}

void buildTilesets(const StoredTileset::list &tilesets, Json::Value &object)
{
    object = Json::arrayValue;

    for (const auto &item : tilesets) {
        buildTileset(item, object.append(Json::nullValue));
    }
}

void buildGlue(const Glue &glue, Json::Value &object)
{
    object = Json::objectValue;
    buildList(glue.id, object["id"]);
    object["dir"] = glue.path;
}

void buildGlues(const Glue::map &glues, Json::Value &object)
{
    object = Json::arrayValue;

    for (const auto &item : glues) {
        buildGlue(item.second, object.append(Json::nullValue));
    }
}

Storage::Properties parse1(const Json::Value &config)
{
    Storage::Properties properties;

    Json::get(properties.referenceFrame, config, "referenceFrame");
    Json::get(properties.revision, config, "revision");

    parseTilesets(properties.tilesets, config["tilesets"]);
    parseGlues(properties.glues, config["glues"]);
    parseTrash(properties.trashBin, config["trashBin"]);

    return properties;
}

void build(Json::Value &config, const Storage::Properties &properties)
{
    config["version"]
        = Json::Int64(detail::CURRENT_JSON_FORMAT_VERSION);

    config["referenceFrame"] = properties.referenceFrame;
    config["revision"] = properties.revision;

    buildTilesets(properties.tilesets, config["tilesets"]);
    buildGlues(properties.glues, config["glues"]);
    buildTrash(properties.trashBin, config["trashBin"]);
}

} // namespace detail

Storage::Properties loadConfig(std::istream &in)
{
    // load json
    Json::Value config;
    Json::Reader reader;
    if (!reader.parse(in, config)) {
        LOGTHROW(err2, vadstena::storage::FormatError)
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

        LOGTHROW(err1, vadstena::storage::FormatError)
            << "Invalid storage config format: unsupported version "
            << version << ".";

    } catch (const Json::Error &e) {
        LOGTHROW(err1, vadstena::storage::FormatError)
            << "Invalid storage config format (" << e.what()
            << "); Unable to work with this storage.";
    }
    throw;
}

void saveConfig(std::ostream &out, const Storage::Properties &properties)
{
    Json::Value config;
    detail::build(config, properties);
    out.precision(15);
    Json::StyledStreamWriter().write(out, config);
}

Storage::Properties loadConfig(const boost::filesystem::path &path)
{
    LOG(info1) << "Loading storage config from " << path  << ".";
    std::ifstream f;
    f.exceptions(std::ios::badbit | std::ios::failbit);
    try {
        f.open(path.string(), std::ios_base::in);
        f.peek();
    } catch (const std::exception &e) {
        LOGTHROW(err1, vadstena::storage::NoSuchStorage)
            << "Unable to load config file " << path << ".";
    }
    auto p(loadConfig(f));
    f.close();
    return p;
}

void saveConfig(const boost::filesystem::path &path
                , const Storage::Properties &properties)
{
    LOG(info1) << "Saving storage config to " << path  << ".";
    std::ofstream f;
    f.exceptions(std::ios::badbit | std::ios::failbit);
    f.open(path.string(), std::ios_base::out);
    saveConfig(f, properties);
    f.close();
}

namespace detail_extra {

ExtraStorageProperties parse1(const Json::Value &config)
{
    ExtraStorageProperties ep;

    if (config.isMember("position")) {
        ep.position = boost::in_place();
        *ep.position = registry::positionFromJson(config["position"]);
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

    // browser config options -- whole JSON object held in opaque pointer.
    if (config.isMember("browserOptions")) {
        const auto &bco(config["browserOptions"]);
        Json::check(bco, Json::objectValue);
        ep.browserOptions = std::make_shared<BrowserOptions>(bco);
    }

    return ep;
}

void build(Json::Value &config, const ExtraStorageProperties &ep)
{
    if (ep.position) {
        config["position"] = registry::asJson(*ep.position);
    }

    if (!ep.credits.empty()) {
        config["credits"] = registry::asJson(ep.credits);
    }

    if (!ep.boundLayers.empty()) {
        config["boundLayers"] = registry::asJson(ep.boundLayers);
    }

    if (!ep.rois.empty()) {
        config["rois"] = registry::asJson(ep.rois);
    }

    if (!ep.namedViews.empty()) {
        registry::BoundLayer::dict tmp;
        config["namedViews"]
            = registry::asJson(ep.namedViews, tmp);
    }

    if (ep.view) {
        registry::BoundLayer::dict tmp;
        config["view"] = registry::asJson(ep.view, tmp);
    }

    if (ep.browserOptions) {
        config["browserOptions"] = ep.browserOptions->value;
    }
}

} // namespace detail_extra

ExtraStorageProperties
extraStorageConfigFromJson(int version, const Json::Value &config)
{
    try {
        switch (version) {
        case 1:
            return detail_extra::parse1(config);
        }

        LOGTHROW(err1, vadstena::storage::FormatError)
            << "Invalid extra config format: unsupported version "
            << version << ".";

    } catch (const Json::Error &e) {
        LOGTHROW(err1, vadstena::storage::FormatError)
            << "Invalid extra config format (" << e.what()
            << "); Unable to work with this config.";
    }
    throw;
}

ExtraStorageProperties loadExtraConfig(std::istream &in)
{
    // load json
    Json::Value config;
    Json::Reader reader;
    if (!reader.parse(in, config)) {
        LOGTHROW(err2, vadstena::storage::FormatError)
            << "Unable to parse extra config: "
            << reader.getFormattedErrorMessages() << ".";
    }

    int version(0);
    try {
        Json::get(version, config, "version");
    } catch (const Json::Error &e) {
        LOGTHROW(err1, vadstena::storage::FormatError)
            << "Invalid extra config format (" << e.what()
            << "); Unable to work with this config.";
    }

    return extraStorageConfigFromJson(version, config);
}

ExtraStorageProperties loadExtraConfig(const boost::filesystem::path &path)
{
    LOG(info1) << "Loading storage extra config from " << path  << ".";
    std::ifstream f;
    f.exceptions(std::ios::badbit | std::ios::failbit);
    try {
        f.open(path.string(), std::ios_base::in);
    } catch (const std::exception &e) {
        return {};
    }
    auto p(loadExtraConfig(f));
    f.close();
    return p;
}

void extraStorageConfigToJson(Json::Value &config
                              , const ExtraStorageProperties &properties)
{
    detail_extra::build(config, properties);
}

} } } // namespace vadstena::vts::storage
