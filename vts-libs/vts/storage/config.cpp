#include <boost/lexical_cast.hpp>
#include "dbglog/dbglog.hpp"
#include "jsoncpp/as.hpp"

#include "./config.hpp"
#include "./detail.hpp"
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
    std::string s;
    Json::get(s, value, "mode");
    tileset.glueMode = boost::lexical_cast<StoredTileset::GlueMode>(s);
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
    object["mode"] = boost::lexical_cast<std::string>(tileset.glueMode);
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
    LOG(info1) << "Loading config from " << path  << ".";
    std::ifstream f;
    f.exceptions(std::ios::badbit | std::ios::failbit);
    try {
        f.open(path.string(), std::ios_base::in);
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
    LOG(info1) << "Saving config to " << path  << ".";
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
        ep.credits = registry::creditsFromJson(config["extraCredits"]);
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

    try {
        int version(0);
        Json::get(version, config, "version");

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

ExtraStorageProperties loadExtraConfig(const boost::filesystem::path &path)
{
    LOG(info1) << "Loading extra config from " << path  << ".";
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

} } } // namespace vadstena::vts::storage
