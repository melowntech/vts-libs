/**
 * Copyright (c) 2017 Melown Technologies SE
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * *  Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#include <iterator>

#include <boost/uuid/string_generator.hpp>
#include <boost/uuid/uuid_io.hpp>

#include "dbglog/dbglog.hpp"
#include "utility/base64.hpp"
#include "jsoncpp/as.hpp"
#include "jsoncpp/io.hpp"

#include "config.hpp"
#include "detail.hpp"
#include "driver.hpp"
#include "../../storage/error.hpp"
#include "../../registry/json.hpp"

namespace vtslibs { namespace vts { namespace tileset {

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

driver::PlainOptions parsePlainDriver(const Json::Value &value)
{
    driver::PlainOptions driverOptions;

    int tmp;
    Json::get(tmp, value, "binaryOrder");
    driverOptions.binaryOrder(tmp);

    if (value.isMember("metaUnusedBits")) {
        // new
        Json::get(tmp, value, "metaUnusedBits");
        driverOptions.metaUnusedBits(tmp);
    }

    std::string uuid;
    Json::get(uuid, value, "uuid");
    driverOptions.uuid(boost::uuids::string_generator()(uuid));

    return driverOptions;
}

driver::AggregatedOptions parseAggregatedDriver(const Json::Value &value)
{
    // optimized aggregated driver
    driver::AggregatedOptions driverOptions;

    std::string storagePath;
    Json::get(storagePath, value, "storage");
    driverOptions.storagePath = storagePath;

    parseIdArray(std::inserter(driverOptions.tilesets
                               , driverOptions.tilesets.begin())
                 , value, "tilesets");
    Json::get(driverOptions.tsMap, value, "tsMap");

    driverOptions.tsMap = utility::base64::decode(driverOptions.tsMap);

    if (value.isMember("surfaceReferences")) {
        Json::get(driverOptions.surfaceReferences, value, "surfaceReferences");
    } else {
        // force default
        driverOptions.surfaceReferences = true;
    }

    auto mo(value["metaOptions"]);
    if (!mo.isNull()) {
        driverOptions.metaOptions = parsePlainDriver(mo);
        Json::get(driverOptions.staticMetaRange.min, mo, "lodRange", 0);
        Json::get(driverOptions.staticMetaRange.max, mo, "lodRange", 1);
    }

    return driverOptions;
}

driver::RemoteOptions parseRemoteDriver(const Json::Value &value)
{
    driver::RemoteOptions driverOptions;

    Json::get(driverOptions.url, value, "url");

    return driverOptions;
}

driver::LocalOptions parseLocalDriver(const Json::Value &value)
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

void buildDriver(const driver::PlainOptions &options, Json::Value &value)
{
    value["binaryOrder"] = options.binaryOrder();
    value["metaUnusedBits"] = options.metaUnusedBits();
    value["uuid"] = to_string(options.uuid());
}

Json::Value buildDriver(const boost::any &d)
{
    Json::Value value(Json::objectValue);

    if (auto opts = boost::any_cast<const driver::PlainOptions>(&d)) {
        value["type"] = "plain";
        buildDriver(*opts, value);
        return value;
    } else if (auto opts = boost::any_cast
               <const driver::AggregatedOptions>(&d))
    {
        value["type"] = "aggregated";
        value["storage"] = opts->storagePath.string();
        value["tilesets"] = buildIdArray(opts->tilesets.begin()
                                         , opts->tilesets.end());
        value["tsMap"] = utility::base64::encode(opts->tsMap);
        value["surfaceReferences"] = opts->surfaceReferences;

        if (opts->metaOptions) {
            auto &mo(value["metaOptions"] = Json::objectValue);
            buildDriver(*opts->metaOptions, mo);

            auto &lodRange(mo["lodRange"] = Json::arrayValue);
            lodRange.append(opts->staticMetaRange.min);
            lodRange.append(opts->staticMetaRange.max);
        }

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

    if (config.isMember("mergeBottomLod")) {
        Json::get(properties.mergeBottomLod, config, "mergeBottomLod");
    } else {
        properties.mergeBottomLod = 0;
    }

    if (config.isMember("nominalTexelSize")) {
        properties.nominalTexelSize = boost::in_place();
        Json::get(*properties.nominalTexelSize, config, "nominalTexelSize");
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

    // left for compatilibily reasons, remove when all vts instances are
    // upgraded
    config["spatialDivisionExtents"] = Json::objectValue;

    if (properties.mergeBottomLod) {
        config["mergeBottomLod"] = properties.mergeBottomLod;
    }

    if (properties.nominalTexelSize) {
        config["nominalTexelSize"] = *properties.nominalTexelSize;
    }
}

} // namespace detail

FullTileSetProperties loadConfig(std::istream &in
                                 , const boost::filesystem::path &path)
{
    // load json
    auto config(Json::read<storage::FormatError>
                (in, path, "tileset config"));

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
    Json::write(out, config);
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

    if (config.isMember("bodies")) {
        ep.bodies = registry::bodiesFromJson(config["bodies"]);
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

ExtraTileSetProperties loadExtraConfig(std::istream &in
                                       , const boost::filesystem::path &path)
{
    // load json
    auto config(Json::read<storage::FormatError>
                (in, path, "tileset extra config"));

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
    auto p(loadExtraConfig(f, path));
    f.close();
    return p;
}

boost::optional<unsigned int> loadRevision(std::istream &in)
{
    // load json
    Json::Value config;
    if (!Json::read(in, config)) {
        return boost::none;
    }

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

void saveDriver(std::ostream &out, const boost::any &driver)
{
    out.precision(15);
    Json::Value config(detail::buildDriver(driver));
    Json::write(out, config);
}

void saveDriver(const boost::filesystem::path &path
                , const boost::any &driver)
{
    LOG(info1) << "Saving driver config to " << path  << ".";
    std::ofstream f;
    f.exceptions(std::ios::badbit | std::ios::failbit);
    f.open(path.string(), std::ios_base::out);
    saveDriver(f, driver);
    f.close();
}

boost::any loadDriver(std::istream &in, const boost::filesystem::path &path)
{
    auto config(Json::read<storage::FormatError>
                (in, path, "tileset driver options"));
    return detail::parseDriver(config);
}

boost::any loadDriver(const boost::filesystem::path &path)
{
    LOG(info1) << "Loading driver config from " << path  << ".";
    std::ifstream f;
    f.exceptions(std::ios::badbit | std::ios::failbit);
    f.open(path.string(), std::ios_base::in);
    const auto res(loadDriver(f, path));
    f.close();
    return res;
}

} } } // namespace vtslibs::vts::tileset
