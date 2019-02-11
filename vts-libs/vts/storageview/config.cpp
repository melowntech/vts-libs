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
#include <boost/filesystem.hpp>

#include "dbglog/dbglog.hpp"
#include "jsoncpp/as.hpp"
#include "jsoncpp/io.hpp"

#include "config.hpp"
#include "detail.hpp"
#include "../../storage/error.hpp"
#include "../../registry/json.hpp"
#include "../storage/config.hpp"

namespace fs = boost::filesystem;

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

void parseList(std::vector<fs::path> &paths, const Json::Value &value
               , const char *name)
{
    if (!value.isArray()) {
        LOGTHROW(err1, Json::Error)
            << "Type of " << name << " is not a list.";
    }

    for (const auto &element : value) {
        Json::check(element, Json::stringValue);
        paths.push_back(element.asString());
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

    if (config.isMember("include")) {
        detail::parseList(properties.includeMapConfigs, config["include"]
                          , "include");
    }

    return properties;
}

} // namespace detail

StorageView::Properties loadConfig(std::istream &in, const fs::path &path)
{
    // load json
    auto config(Json::read<vtslibs::storage::FormatError>
                (in, path, "storageview config"));

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

StorageView::Properties loadConfig(const fs::path &path)
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
    auto p(loadConfig(f, path));
    f.close();

    p.absolutize(fs::absolute(path).parent_path());
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

void buildList(const std::vector<fs::path> &paths, Json::Value &value)
{
    value = Json::arrayValue;
    for (const auto &path : paths) { value.append(path.string()); }
}

void build(Json::Value &config, const StorageView::Properties &properties)
{
    config["version"]
        = Json::Int64(detail::CURRENT_JSON_FORMAT_VERSION);

    config["storage"] = properties.storagePath.string();
    buildSet(config["tilesets"], properties.tilesets);

    storage::extraStorageConfigToJson(config, properties.extra);

    if (!properties.includeMapConfigs.empty()) {
        detail::buildList(properties.includeMapConfigs, config["include"]);
    }
}

void saveConfig(std::ostream &out, const StorageView::Properties &properties)
{
    Json::Value config;
    detail::build(config, properties);
    out.precision(15);
    Json::write(out, config);
}

} // namespace detail

void saveConfig(const fs::path &path
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

bool checkConfig(const fs::path &path)
{
    LOG(info1) << "Checking storage view config at " << path  << ".";
    std::ifstream f;
    f.open(path.string(), std::ios_base::in);
    f.peek();
    if (!f) { return false; }
    return checkConfig(f);
}

} } } // namespace vtslibs::vts::storageview
