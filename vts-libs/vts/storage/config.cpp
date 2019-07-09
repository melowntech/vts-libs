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
#include <boost/lexical_cast.hpp>
#include "dbglog/dbglog.hpp"
#include "jsoncpp/as.hpp"
#include "jsoncpp/io.hpp"

#include "config.hpp"
#include "detail.hpp"
#include "../../storage/error.hpp"
#include "../../registry/json.hpp"

namespace fs = boost::filesystem;

namespace vtslibs { namespace vts { namespace storage {

namespace detail {

const int CURRENT_JSON_FORMAT_VERSION(1);

void parseList(std::vector<std::string> &ids, const Json::Value &value
               , const char *name)
{
    if (!value.isArray()) {
        LOGTHROW(err1, Json::Error)
            << "Type of " << name << " is not a list.";
    }

    for (const auto &element : value) {
        Json::check(element, Json::stringValue);
        ids.push_back(element.asString());
    }
}

void parseExternalUrl(Proxy2ExternalUrl &proxy2ExternalUrl
                      , const Json::Value &proxied)
{
    for (const auto &name : proxied.getMemberNames()) {
        proxy2ExternalUrl.insert
            (Proxy2ExternalUrl::value_type
             (name, proxied[name].asString()));
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
        // non-versioned -> baseId is same as tilesetId and no version
        tileset.baseId = tileset.tilesetId;
        tileset.version = 0;
    }

    for (const auto &tag : value["tags"]) {
        tileset.tags.insert(tag.asString());
    }

    if (value.isMember("externalUrl")) {
        parseExternalUrl(tileset.proxy2ExternalUrl, value["externalUrl"]);
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

    parseList(glue.id, value["id"], "id");
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

void parseGlues(Glue::IdSet &glues, const Json::Value &value)
{
    if (value.isNull()) { return; }
    if (!value.isArray()) {
        LOGTHROW(err1, Json::Error)
            << "Type of glue set is not an array.";
    }

    for (const auto &element : value) {
        Glue::Id glueId;
        parseList(glueId, element, "id");
        glues.insert(glueId);
    }
}

void parseVirtualSurface(VirtualSurface &virtualSurface
                         , const Json::Value &value)
{
    if (!value.isObject()) {
        LOGTHROW(err1, Json::Error)
            << "Type of virtualSurface is not an object.";
    }

    parseList(virtualSurface.id, value["id"], "id");
    Json::get(virtualSurface.path, value, "dir");
}

void parseVirtualSurfaces(VirtualSurface::map &virtualSurfaces
                          , const Json::Value &value)
{
    if (!value.isArray()) {
        LOGTHROW(err1, Json::Error)
            << "Type of virtualSurfaces is not an array.";
    }

    for (const auto &element : value) {
        Json::check(element, Json::objectValue);
        VirtualSurface g;
        parseVirtualSurface(g, element);
        virtualSurfaces.insert(VirtualSurface::map::value_type(g.id, g));
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

Json::Value buildList(const std::vector<std::string> &ids)
{
    Json::Value value(Json::arrayValue);
    for (const auto &id : ids) { value.append(id); }
    return value;
}

void buildExternalUrl(const Proxy2ExternalUrl &proxy2ExternalUrl
                      , Json::Value &object, const char *name)
{
    if (proxy2ExternalUrl.empty()) { return; }

    auto &proxied(object[name] = Json::objectValue);
    for (const auto &item : proxy2ExternalUrl) {
        proxied[item.first] = item.second;
    }
}

void buildTileset(const StoredTileset &tileset, Json::Value &object)
{
    object = Json::objectValue;
    object["id"] = tileset.tilesetId;
    if (tileset.version > 0) {
        object["base"] = tileset.baseId;
        object["version"] = tileset.version;
    }

    if (!tileset.tags.empty()) {
        auto &tags(object["tags"] = Json::arrayValue);
        for (const auto &tag : tileset.tags) {
            tags.append(tag);
        }
    }

    buildExternalUrl(tileset.proxy2ExternalUrl, object, "externalUrl");
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

void buildGlues(const Glue::IdSet &glues, Json::Value &object)
{
    object = Json::arrayValue;

    for (const auto &id : glues) { object.append(buildList(id)); }
}

void buildVirtualSurface(const VirtualSurface &virtualSurface
                         , Json::Value &object)
{
    object = Json::objectValue;
    buildList(virtualSurface.id, object["id"]);
    object["dir"] = virtualSurface.path;
}

void buildVirtualSurfaces(const VirtualSurface::map &virtualSurfaces
                          , Json::Value &object)
{
    object = Json::arrayValue;

    for (const auto &item : virtualSurfaces) {
        buildVirtualSurface(item.second, object.append(Json::nullValue));
    }
}

Storage::Properties parse1(const Json::Value &config)
{
    Storage::Properties properties;

    Json::get(properties.referenceFrame, config, "referenceFrame");
    Json::get(properties.revision, config, "revision");

    parseTilesets(properties.tilesets, config["tilesets"]);
    parseGlues(properties.glues, config["glues"]);
    parseGlues(properties.pendingGlues, config["glues.pending"]);
    parseGlues(properties.emptyGlues, config["glues.empty"]);
    if (config.isMember("virtualSurfaces")) {
        parseVirtualSurfaces
            (properties.virtualSurfaces
             , config["virtualSurfaces"]);
    }
    parseTrash(properties.trashBin, config["trashBin"]);

    if (config.isMember("glues.externalUrl")) {
        parseExternalUrl(properties.gluesExternalUrl
                         , config["glues.externalUrl"]);
    }

    if (config.isMember("virtualSurfaces.externalUrl")) {
        parseExternalUrl(properties.vsExternalUrl
                         , config["virtualSurfaces.externalUrl"]);
    }

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
    buildGlues(properties.pendingGlues, config["glues.pending"]);
    buildGlues(properties.emptyGlues, config["glues.empty"]);
    buildVirtualSurfaces(properties.virtualSurfaces
                         , config["virtualSurfaces"]);
    buildTrash(properties.trashBin, config["trashBin"]);

    buildExternalUrl(properties.gluesExternalUrl, config, "glues.externalUrl");
    buildExternalUrl(properties.vsExternalUrl, config
                     , "virtualSurfaces.externalUrl");
}

} // namespace detail

Storage::Properties
loadConfig(std::istream &in, const boost::filesystem::path &path)
{
    // load json
    auto config(Json::read<vtslibs::storage::FormatError>
                (in, path, "storage config"));

    try {
        int version(0);
        Json::get(version, config, "version");

        switch (version) {
        case 1:
            return detail::parse1(config);
        }

        LOGTHROW(err1, vtslibs::storage::FormatError)
            << "Invalid storage config format: unsupported version "
            << version << ".";

    } catch (const Json::Error &e) {
        LOGTHROW(err1, vtslibs::storage::FormatError)
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
    Json::write(out, config);
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
        LOGTHROW(err1, vtslibs::storage::NoSuchStorage)
            << "Unable to load config file " << path << ".";
    }
    auto p(loadConfig(f, path));
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

    Json::getOpt(ep.virtualSurfacesEnabled, config, "virtualSurfacesEnabled");

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

void build(Json::Value &config, const ExtraStorageProperties &ep)
{
    // defaults to true, only "false" is interesting here
    if (!ep.virtualSurfacesEnabled) {
        config["virtualSurfacesEnabled"] = false;
    }

    if (ep.position) {
        config["position"] = registry::asJson(*ep.position);
    }

    if (!ep.credits.empty()) {
        config["credits"] = registry::asJson(ep.credits);
    }

    if (!ep.boundLayers.empty()) {
        config["boundLayers"] = registry::asJson(ep.boundLayers);
    }

    if (!ep.freeLayers.empty()) {
        config["freeLayers"] = registry::asJson(ep.freeLayers);
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

    if (!ep.bodies.empty()) {
        config["bodies"] == registry::asJson(ep.bodies);
    }

    if (!ep.browserOptions.empty()) {
        try {
            config["browserOptions"]
                = boost::any_cast<Json::Value>(ep.browserOptions);
        } catch (const boost::bad_any_cast&) {
            // ignore
        }
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

        LOGTHROW(err1, vtslibs::storage::FormatError)
            << "Invalid extra config format: unsupported version "
            << version << ".";

    } catch (const Json::Error &e) {
        LOGTHROW(err1, vtslibs::storage::FormatError)
            << "Invalid extra config format (" << e.what()
            << "); Unable to work with this config.";
    }
    throw;
}

ExtraStorageProperties loadExtraConfig(std::istream &in
                                       , const boost::filesystem::path &path)
{
    // load json
    auto config(Json::read<vtslibs::storage::FormatError>
                (in, path, "storage extra config"));

    int version(0);
    try {
        Json::get(version, config, "version");
    } catch (const Json::Error &e) {
        LOGTHROW(err1, vtslibs::storage::FormatError)
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
    auto p(loadExtraConfig(f, path));
    f.close();
    return p;
}

void extraStorageConfigToJson(Json::Value &config
                              , const ExtraStorageProperties &properties)
{
    detail_extra::build(config, properties);
}

} } } // namespace vtslibs::vts::storage
