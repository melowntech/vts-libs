#include <boost/lexical_cast.hpp>

#include "dbglog/dbglog.hpp"

#include "jsoncpp/as.hpp"

#include "./freelayer.hpp"
#include "./json.hpp"

namespace vadstena { namespace registry {

constexpr char FreeLayer::typeName[];

namespace {

void build(Json::Value &content, const FreeLayer::Geodata &def)
{
    (void) content;
    (void) def;
    // TODO implement me
}

void build(Json::Value &content, const FreeLayer::GeodataTiles &def)
{
    (void) content;
    (void) def;
    // TODO implement me
}

void build(Json::Value &content, const FreeLayer::MeshTiles &def)
{
    auto &lodRange(content["lodRange"] = Json::arrayValue);
    lodRange.append(def.lodRange.min);
    lodRange.append(def.lodRange.max);

    auto &tileRange(content["tileRange"] = Json::arrayValue);
    auto &tileRangeMin(tileRange.append(Json::arrayValue));
    tileRangeMin.append(def.tileRange.ll(0));
    tileRangeMin.append(def.tileRange.ll(1));
    auto &tileRangeMax(tileRange.append(Json::arrayValue));
    tileRangeMax.append(def.tileRange.ur(0));
    tileRangeMax.append(def.tileRange.ur(1));

    content["metaUrl"] = def.metaUrl;
    content["meshUrl"] = def.meshUrl;
    content["textureUrl"] = def.textureUrl;
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
    (void) def;
    (void) content;
    // TODO implement me
}

void parse(FreeLayer::GeodataTiles &def, const Json::Value &content)
{
    (void) def;
    (void) content;
    // TODO implement me
}

void parse(FreeLayer::MeshTiles &def, const Json::Value &content)
{
    Json::get(def.lodRange.min, content, "lodRange", 0);
    Json::get(def.lodRange.max, content, "lodRange", 1);

    def.tileRange = tileRangeFromJson(content["tileRange"]);
    Json::get(def.metaUrl, content, "metaUrl");
    Json::get(def.meshUrl, content, "meshUrl");
    Json::get(def.textureUrl, content, "textureUrl");
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

void fromJson(FreeLayer::dict &freeLayers, const Json::Value &value)
{
    parse(freeLayers, value);
}

void saveFreeLayer(std::ostream &out, const FreeLayer &freeLayer)
{
    Json::Value content;
    build(content, freeLayer);
    out.precision(15);
    Json::StyledStreamWriter().write(out, content);
}

} } // namespace vadstena::registry
