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

#include "freelayer.hpp"
#include "json.hpp"

namespace vtslibs { namespace registry {

constexpr char FreeLayer::typeName[];

namespace {

inline void parse(math::Extents3 &extents, const Json::Value &content)
{
    get(extents.ll(0), content, "ll", 0);
    get(extents.ll(1), content, "ll", 1);
    get(extents.ll(2), content, "ll", 2);
    get(extents.ur(0), content, "ur", 0);
    get(extents.ur(1), content, "ur", 1);
    get(extents.ur(2), content, "ur", 2);
}

inline void parse(boost::any &options, const Json::Value &content)
{
    if (content.isMember("options")) { options = content["options"]; }
}

inline void build(Json::Value &content, const math::Extents3 &extents)
{
    content = Json::objectValue;

    auto &ll(content["ll"] = Json::arrayValue);
    ll.append(extents.ll(0));
    ll.append(extents.ll(1));
    ll.append(extents.ll(2));

    auto &ur(content["ur"] = Json::arrayValue);
    ur.append(extents.ur(0));
    ur.append(extents.ur(1));
    ur.append(extents.ur(2));
}

inline void build(Json::Value &content, const TileRange &tileRange)
{
    content = Json::arrayValue;
    auto &tileRangeMin(content.append(Json::arrayValue));
    tileRangeMin.append(tileRange.ll(0));
    tileRangeMin.append(tileRange.ll(1));
    auto &tileRangeMax(content.append(Json::arrayValue));
    tileRangeMax.append(tileRange.ur(0));
    tileRangeMax.append(tileRange.ur(1));
}

inline void build(Json::Value &content, const LodRange &lodRange)
{
    content = Json::arrayValue;
    content.append(lodRange.min);
    content.append(lodRange.max);
}

inline void build(Json::Value &content, const boost::any &options)
{
    if (options.empty()) { return; }
    content["options"] = boost::any_cast<Json::Value>(options);
}

void build(Json::Value &content, const FreeLayer::Geodata &def)
{
    build(content["extents"], def.extents);
    content["displaySize"] = def.displaySize;
    content["label"] = def.label;
    content["geodata"] = def.geodata;
    content["style"] = def.style;
    build(content, def.options);
}

void build(Json::Value &content, const FreeLayer::GeodataTiles &def)
{
    build(content["lodRange"], def.lodRange);
    build(content["tileRange"], def.tileRange);
    if (def.displaySize > 0) { content["displaySize"] = def.displaySize; }

    content["metaUrl"] = def.metaUrl;
    content["geodataUrl"] = def.geodataUrl;
    content["style"] = def.style;
    build(content, def.options);
}

void build(Json::Value &content, const FreeLayer::MeshTiles &def)
{
    build(content["lodRange"], def.lodRange);
    build(content["tileRange"], def.tileRange);
    content["metaUrl"] = def.metaUrl;
    content["meshUrl"] = def.meshUrl;
    content["textureUrl"] = def.textureUrl;
    build(content, def.options);
}

void build(Json::Value &content, const FreeLayer &fl
           , bool inlineCredits = true)
{
    if (fl.type == FreeLayer::Type::external) {
        // just url
        content = boost::get<std::string>(fl.definition);
        return;
    }

    content = Json::objectValue;
    content["type"] = boost::lexical_cast<std::string>(fl.type);

    content["credits"] = asJson(fl.credits, inlineCredits);

    switch (fl.type) {
    case FreeLayer::Type::external: break; // handled above

    case FreeLayer::Type::geodata:
        build(content, boost::get<FreeLayer::Geodata>(fl.definition));
        break;

    case FreeLayer::Type::geodataTiles:
        build(content, boost::get<FreeLayer::GeodataTiles>(fl.definition));
        break;

    case FreeLayer::Type::meshTiles:
        build(content, boost::get<FreeLayer::MeshTiles>(fl.definition));
        break;
    }
}

void build(Json::Value &content, const FreeLayer::dict &fls
           , bool inlineCredits = true)
{
    content = Json::objectValue;

    for (const auto &item : fls) {
        const auto &id(item.first);
        const auto &fl(item.second);

        // regular free layer
        build(content[id], fl, inlineCredits);
    }
}

void parse(FreeLayer::Geodata &def, const Json::Value &content)
{
    parse(def.extents, content["extents"]);
    Json::get(def.displaySize, content, "displaySize");
    Json::get(def.label, content, "label");
    Json::get(def.geodata, content, "geodata");
    Json::get(def.style, content, "style");
    parse(def.options, content);
}

void parse(FreeLayer::GeodataTiles &def, const Json::Value &content)
{
    Json::get(def.lodRange.min, content, "lodRange", 0);
    Json::get(def.lodRange.max, content, "lodRange", 1);
    def.tileRange = tileRangeFromJson(content["tileRange"]);
    if (!Json::getOpt(def.displaySize, content, "displaySize")) {
        // compatibility
        def.displaySize = 0;
    }

    Json::get(def.metaUrl, content, "metaUrl");
    Json::get(def.geodataUrl, content, "geodataUrl");
    Json::get(def.style, content, "style");
    parse(def.options, content);
}

void parse(FreeLayer::MeshTiles &def, const Json::Value &content)
{
    Json::get(def.lodRange.min, content, "lodRange", 0);
    Json::get(def.lodRange.max, content, "lodRange", 1);

    def.tileRange = tileRangeFromJson(content["tileRange"]);
    Json::get(def.metaUrl, content, "metaUrl");
    Json::get(def.meshUrl, content, "meshUrl");
    Json::get(def.textureUrl, content, "textureUrl");
    parse(def.options, content);
}

void parse(FreeLayer &fl, const Json::Value &content)
{
    if (content.isString()) {
        fl.type = FreeLayer::Type::external;
        fl.definition = content.asString();
        return;
    }

    std::string s;
    fl.type = boost::lexical_cast<FreeLayer::Type>
        (Json::get(s, content, "type"));

    fromJson(fl.credits, content["credits"]);

    switch (fl.type) {
    case FreeLayer::Type::external: break; // already handled above

    case FreeLayer::Type::geodata:
        parse(fl.createDefinition<FreeLayer::Geodata>(), content);
        break;

    case FreeLayer::Type::geodataTiles:
        parse(fl.createDefinition<FreeLayer::GeodataTiles>(), content);
        break;

    case FreeLayer::Type::meshTiles:
        parse(fl.createDefinition<FreeLayer::MeshTiles>(), content);
        break;
    }
}

void parse(FreeLayer::dict &fls, const Json::Value &content)
{
    for (const auto &id : Json::check(content, Json::objectValue)
             .getMemberNames())
    {
        try {
            FreeLayer fl;
            fl.id = id;
            parse(fl, content[id]);
            fls.set(id, fl);
        } catch (const Json::Error &e) {
            LOGTHROW(err1, storage::FormatError)
                << "Invalid free layers file format (" << e.what()
                << ").";
        }
    }
}

} // namespace

Json::Value asJson(const FreeLayer::dict &freeLayers
                   , bool inlineCredits)
{
    Json::Value content;
    build(content, freeLayers, inlineCredits);
    return content;
}

Json::Value asJson(const FreeLayer &freeLayer
                   , bool inlineCredits)
{
    Json::Value content;
    build(content, freeLayer, inlineCredits);
    return content;
}

void fromJson(FreeLayer::dict &freeLayers, const Json::Value &value)
{
    parse(freeLayers, value);
}

void saveFreeLayer(std::ostream &out, const FreeLayer &freeLayer)
{
    Json::Value content;
    build(content, freeLayer);
    out.precision(15);
    Json::write(out, content);
}

FreeLayer loadFreeLayer(std::istream &in
                        , const boost::filesystem::path &path)
{
    // load json
    auto content(Json::read<storage::FormatError>
                 (in, path, "free layer"));

    FreeLayer freeLayer;
    parse(freeLayer, content);
    return freeLayer;
}

namespace {

struct Absolutize
    : boost::static_visitor<>
{
    Absolutize(const utility::Uri &base) : base(base) {}

    const utility::Uri &base;

    void absolutize(std::string &url) const {
        url = base.resolve(url).str();
    }

    void absolutize(boost::optional<std::string> &url) const {
        if (url) { absolutize(*url); }
    }

    void absolutize(FreeLayer &fl) const {
        boost::apply_visitor(*this, fl.definition);
    }

    // FreeLayer::definition visitor:

    void operator()(std::string &def) const {
        absolutize(def);
    }

    void operator()(registry::FreeLayer::Geodata &def) const {
        absolutize(def.geodata);
        absolutize(def.style);
    }

    void operator()(registry::FreeLayer::GeodataTiles &def) const {
        absolutize(def.metaUrl);
        absolutize(def.geodataUrl);
        absolutize(def.style);
    }

    void operator()(registry::FreeLayer::MeshTiles &def) const {
        absolutize(def.metaUrl);
        absolutize(def.meshUrl);
        absolutize(def.textureUrl);
    }
};

} // namespace

FreeLayer absolutize(const FreeLayer &freeLayer, const utility::Uri &baseUrl)
{
    auto fl(freeLayer);
    Absolutize(baseUrl).absolutize(fl);
    return fl;
}

} } // namespace vtslibs::registry
