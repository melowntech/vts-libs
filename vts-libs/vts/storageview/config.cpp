#include <boost/lexical_cast.hpp>
#include <boost/filesystem.hpp>

#include "dbglog/dbglog.hpp"
#include "jsoncpp/as.hpp"

#include "./config.hpp"
#include "./detail.hpp"
#include "../../storage/error.hpp"
#include "../../registry/json.hpp"
#include "../storage/config.hpp"

namespace vtslibs { namespace vts { namespace storageview {

namespace detail {

const int CURRENT_JSON_FORMAT_VERSION(1);

void parseSet(std::set<std::string> &ids, const Json::Value &object
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

StorageView::Properties parse1(const Json::Value &config)
{
    StorageView::Properties properties;
    std::string stmp;
    Json::get(stmp, config, "storage");
    properties.storagePath = stmp;
    parseSet(properties.tilesets, config, "tilesets");
    if (config.isMember("freeLayerTilesets")) {
        parseSet(properties.freeLayerTilesets, config, "freeLayerTilesets");
    }

    // let storage stuff parse the extra configuration
    properties.extra = storage::extraStorageConfigFromJson(1, config);

    return properties;
}

} // namespace detail

StorageView::Properties loadConfig(std::istream &in)
{
    // load json
    Json::Value config;
    Json::Reader reader;
    if (!reader.parse(in, config)) {
        LOGTHROW(err1, vtslibs::storage::FormatError)
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

        LOGTHROW(err1, vtslibs::storage::FormatError)
            << "Invalid storageview config format: unsupported version "
            << version << ".";

    } catch (const Json::Error &e) {
        LOGTHROW(err1, vtslibs::storage::FormatError)
            << "Invalid storageview config format (" << e.what()
            << "); Unable to work with this storageview.";
    }
    throw;
}

StorageView::Properties loadConfig(const boost::filesystem::path &path)
{
    LOG(info1) << "Loading storage view config from " << path  << ".";
    std::ifstream f;
    f.exceptions(std::ios::badbit | std::ios::failbit);
    try {
        f.open(path.string(), std::ios_base::in);
        f.peek();
    } catch (const std::exception &e) {
        LOGTHROW(err1, vtslibs::storage::NoSuchStorageView)
            << "Unable to load storage view config file " << path << ".";
    }
    auto p(loadConfig(f));
    // fix path
    p.storagePath = boost::filesystem::absolute
        (p.storagePath, boost::filesystem::absolute(path).parent_path());
    f.close();
    return p;
}

namespace detail {

void buildSet(Json::Value &object, const std::set<std::string> &ids)
{
    object = Json::arrayValue;
    for (const auto &id : ids) {
        object.append(id);
    }
}

void build(Json::Value &config, const StorageView::Properties &properties)
{
    config["version"]
        = Json::Int64(detail::CURRENT_JSON_FORMAT_VERSION);

    config["storage"] = properties.storagePath.string();
    buildSet(config["tilesets"], properties.tilesets);

    storage::extraStorageConfigToJson(config, properties.extra);
}

void saveConfig(std::ostream &out, const StorageView::Properties &properties)
{
    Json::Value config;
    detail::build(config, properties);
    out.precision(15);
    Json::StyledStreamWriter().write(out, config);
}

} // namespace detail

void saveConfig(const boost::filesystem::path &path
                , const StorageView::Properties &properties)
{
    LOG(info1) << "Saving storage view config to " << path  << ".";
    std::ofstream f;
    f.exceptions(std::ios::badbit | std::ios::failbit);
    f.open(path.string(), std::ios_base::out);
    detail::saveConfig(f, properties);
    f.close();
}

bool checkConfig(std::istream &in)
{
    try {
        in.exceptions(std::ios::badbit | std::ios::failbit);
        loadConfig(in);
    } catch (const vtslibs::storage::Error&) {
        return false;
    }
    return true;
}

bool checkConfig(const boost::filesystem::path &path)
{
    LOG(info1) << "Checking storage view config at " << path  << ".";
    std::ifstream f;
    f.open(path.string(), std::ios_base::in);
    f.peek();
    if (!f) { return false; }
    return checkConfig(f);
}

} } } // namespace vtslibs::vts::storageview
