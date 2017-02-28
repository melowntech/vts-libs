#include <typeinfo>
#include <cstdint>

#include "dbglog/dbglog.hpp"
#include "jsoncpp/as.hpp"

#include "./json.hpp"
#include "../storage/error.hpp"

namespace vtslibs { namespace tilestorage {

namespace detail { namespace tileset {

const int CURRENT_JSON_FORMAT_VERSION(3);

DriverProperties parseDriver(const Json::Value &driver)
{
    DriverProperties dp;
    Json::get(dp.type, driver, "type");
    if (!driver.isMember("options")) { return dp; }

    const auto &options(driver["options"]);
    if (!options.isObject()) {
        LOGTHROW(err1, Json::Error)
            << "Type of driver[options] is not an object.";
    }
    for (const auto &member : options.getMemberNames()) {
        DriverProperties::Options::value_type vt(member, {});
        const auto &v(options[member]);

        switch (v.type()) {
        case Json::ValueType::intValue:
            if (v.asInt64() >= 0) {
                vt.second = std::uint64_t(v.asUInt64());
            } else {
                vt.second = std::int64_t(v.asInt64());
            }
            break;
        case Json::ValueType::uintValue:
            vt.second = std::uint64_t(v.asUInt64()); break;
        case Json::ValueType::realValue: vt.second = v.asDouble(); break;
        case Json::ValueType::stringValue: vt.second = v.asString(); break;
        case Json::ValueType::booleanValue: vt.second = v.asBool(); break;
        default: continue; // ignoring compound types and null
        }

        dp.options.insert(vt);
    }

    return dp;
}

Json::Value buildDriver(const DriverProperties &dp)
{
    Json::Value driver;
    driver["type"] = dp.type;
    if (dp.options.empty()) { return driver; }

    auto &options(driver["options"] = Json::Value(Json::objectValue));
    for (const auto &vt : dp.options) {
        auto &v(options[vt.first]);
        const auto &ti(vt.second.type());

        if (ti == typeid(std::int64_t)) {
            v = Json::Value
                (Json::Int64(boost::any_cast<std::int64_t>(vt.second)));
        } else if (ti == typeid(std::uint64_t)) {
            v = Json::Value
                (Json::UInt64(boost::any_cast<std::uint64_t>(vt.second)));
        } else if (ti == typeid(double)) {
            v = Json::Value(boost::any_cast<double>(vt.second));
        } else if (ti == typeid(std::string)) {
            v = Json::Value(boost::any_cast<std::string>(vt.second));
        } else if (ti == typeid(bool)) {
            v = Json::Value(boost::any_cast<bool>(vt.second));
        } else {
            LOGTHROW(err1, std::runtime_error)
                << "Driver properties: unable to serialize C++ type "
                "with type_info::name <" << ti.name() << ">.";
        }
    }

    return driver;
}

Properties parse1(const Json::Value &config)
{
    Properties properties;
    Json::get(properties.id, config, "id");

    Json::get(properties.foat.lod, config, "foat", 0);
    Json::get(properties.foat.easting, config, "foat", 1);
    Json::get(properties.foat.northing, config, "foat", 2);
    Json::get(properties.foatSize, config, "foat", 3);

    Json::get(properties.metaLevels.lod, config, "meta", 0);
    Json::get(properties.metaLevels.delta, config, "meta", 1);

    Json::get(properties.baseTileSize, config, "baseTileSize");

    for (int i(0); i < 2; ++i) {
        Json::get(properties.alignment(i), config, "alignment", i);
    }

    Json::get(properties.meshTemplate, config, "meshTemplate");
    Json::get(properties.textureTemplate, config, "textureTemplate");
    Json::get(properties.metaTemplate, config, "metaTemplate");

    for (int i(0); i < 3; ++i) {
        Json::get(properties.defaultPosition(i), config, "defaultPosition", i);
    }

    for (int i(0); i < 3; ++i) {
        Json::get(properties.defaultOrientation(i)
                  , config, "defaultOrientation", i);
    }

    Json::get(properties.textureQuality, config, "textureQuality");

    if (config.isMember("driver")) {
        properties.driver = parseDriver(config["driver"]);
    }

    return properties;
}

Properties parse2(const Json::Value &config)
{
    auto properties(parse1(config));

    Json::get(properties.srs, config, "srs");

    return properties;
}

Properties parse3(const Json::Value &config)
{
    auto properties(parse2(config));

    Json::get(properties.texelSize, config, "texelSize");

    return properties;
}

} } // namespace detail::tileset

void parse(Properties &properties, const Json::Value &config)
{
    try {
        int version(0);
        Json::get(version, config, "version");

        switch (version) {
        case 1:
            properties = detail::tileset::parse1(config);
            return;

        case 2:
            properties = detail::tileset::parse2(config);
            return;

        case 3:
            properties = detail::tileset::parse3(config);
            return;
        }

        LOGTHROW(err1, storage::FormatError)
            << "Invalid tileset config format: unsupported version "
            << version << ".";

    } catch (const Json::Error &e) {
        LOGTHROW(err1, storage::FormatError)
            << "Invalid tileset config format (" << e.what()
            << "); Unable to work with this tileset.";
    }
}

void build(Json::Value &config, const Properties &properties)
{
    config["version"]
        = Json::Int64(detail::tileset::CURRENT_JSON_FORMAT_VERSION);

    config["id"] = properties.id;

    auto &foat(config["foat"] = Json::Value(Json::arrayValue));
    foat.append(Json::Int64(properties.foat.lod));
    foat.append(Json::Int64(properties.foat.easting));
    foat.append(Json::Int64(properties.foat.northing));
    foat.append(Json::UInt64(properties.foatSize));

    auto &meta(config["meta"] = Json::Value(Json::arrayValue));
    meta.append(Json::Int64(properties.metaLevels.lod));
    meta.append(Json::UInt64(properties.metaLevels.delta));

    config["baseTileSize"] = Json::UInt64(properties.baseTileSize);

    auto &alignment
        (config["alignment"] = Json::Value(Json::arrayValue));
    alignment.append(Json::Int64(properties.alignment(0)));
    alignment.append(Json::Int64(properties.alignment(1)));

    config["srs"] = properties.srs;

    config["meshTemplate"] = properties.meshTemplate;
    config["textureTemplate"] = properties.textureTemplate;
    config["metaTemplate"] = properties.metaTemplate;

    auto &defaultPosition
        (config["defaultPosition"] = Json::Value(Json::arrayValue));
    defaultPosition.append(properties.defaultPosition(0));
    defaultPosition.append(properties.defaultPosition(1));
    defaultPosition.append(properties.defaultPosition(2));

    auto &defaultOrientation
        (config["defaultOrientation"] = Json::Value(Json::arrayValue));
    defaultOrientation.append(properties.defaultOrientation(0));
    defaultOrientation.append(properties.defaultOrientation(1));
    defaultOrientation.append(properties.defaultOrientation(2));

    config["textureQuality"] = Json::Int(properties.textureQuality);
    config["texelSize"] = Json::Value(properties.texelSize);

    config["driver"] = detail::tileset::buildDriver(properties.driver);
}

namespace detail { namespace storage {

const int CURRENT_JSON_FORMAT_VERSION(1);

Json::Value buildTileSetDescriptor(const TileSetDescriptor &desc)
{
    Json::Value set(Json::objectValue);

    set["type"] = desc.locator.type;
    set["location"] = desc.locator.location;

    return set;
}

TileSetDescriptor parseTileSetDescriptor(const Json::Value &tileSet)
{
    TileSetDescriptor desc;
    Json::get(desc.locator.type, tileSet, "type");
    Json::get(desc.locator.location, tileSet, "location");

    return desc;
}

StorageProperties parse1(const Json::Value &config)
{
    StorageProperties p;
    p.outputSet = parseTileSetDescriptor
        (Json::check(config["output"], Json::objectValue));

    const auto &input(Json::check(config["input"], Json::objectValue));
    for (auto iinput(input.begin()), einput(input.end()); iinput != einput;
         ++iinput)
    {
        p.inputSets[iinput.memberName()] = parseTileSetDescriptor(*iinput);
    }

    return p;
}

} }  // namespace detail::storage

void parse(StorageProperties &properties, const Json::Value &config)
{
    try {
        int version(0);
        Json::get(version, config, "version");

        switch (version) {
        case 1:
            properties = detail::storage::parse1(config);
            return;
        }

        LOGTHROW(err1, storage::FormatError)
            << "Invalid storage config format: unsupported version "
            << version << ".";

    } catch (const Json::Error &e) {
        LOGTHROW(err1, storage::FormatError)
            << "Invalid storage config format (" << e.what()
            << "); Unable to work with this storage.";
    }
}

void build(Json::Value &config, const StorageProperties &properties)
{
    config["version"]
        = Json::Int64(detail::storage::CURRENT_JSON_FORMAT_VERSION);

    config["output"]
        = detail::storage::buildTileSetDescriptor(properties.outputSet);

    auto &input(config["input"] = Json::Value(Json::objectValue));
    for (const auto &inputSet : properties.inputSets) {
        input[inputSet.first]
            = detail::storage::buildTileSetDescriptor(inputSet.second);
    }
}

} } // namespace vtslibs::tilestorage
