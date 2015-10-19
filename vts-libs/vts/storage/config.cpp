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

void parseGlue(Glue &glue, const Json::Value &value)
{
    if (!value.isObject()) {
        LOGTHROW(err1, Json::Error)
            << "Type of glue is not an object.";
    }

    parseList(glue.id, value, "id");
    Json::get(glue.path, value, "dir");
}

void parseGlues(Glue::list &glues, const Json::Value &value)
{
    if (!value.isArray()) {
        LOGTHROW(err1, Json::Error)
            << "Type of glues is not an array.";
    }

    for (const auto &element : value) {
        Json::check(element, Json::objectValue);
        glues.push_back({});
        parseGlue(glues.back(), element);
    }
}

void buildList(const std::vector<std::string> &ids, Json::Value &value)
{
    value = Json::arrayValue;
    for (const auto &id : ids) { value.append(id); }
}

void buildGlue(const Glue &glue, Json::Value &object)
{
    object = Json::objectValue;
    buildList(glue.id, object["id"]);
    object["dir"] = glue.path;
}

void buildGlues(const Glue::list &glues, Json::Value &object)
{
    object = Json::arrayValue;

    for (const auto &glue : glues) {
        buildGlue(glue, object.append(Json::nullValue));
    }
}

Storage::Properties parse1(const Json::Value &config)
{
    Storage::Properties properties;

    Json::get(properties.referenceFrame, config, "referenceFrame");
    Json::get(properties.revision, config, "revision");

    parseList(properties.tilesets, config, "tilesets");
    parseGlues(properties.glues, config["glues"]);

    return properties;
}

void build(Json::Value &config, const Storage::Properties &properties)
{
    config["version"]
        = Json::Int64(detail::CURRENT_JSON_FORMAT_VERSION);

    config["referenceFrame"] = properties.referenceFrame;
    config["revision"] = properties.revision;

    buildList(properties.tilesets, config["tilesets"]);
    buildGlues(properties.glues, config["glues"]);
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
    (void) config;
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
        LOGTHROW(err1, vadstena::storage::NoSuchStorage)
            << "Unable to load extra config file " << path << ".";
    }
    auto p(loadExtraConfig(f));
    f.close();
    return p;
}

} } } // namespace vadstena::vts::storage
