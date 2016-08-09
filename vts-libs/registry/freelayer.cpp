#include <boost/lexical_cast.hpp>

#include "dbglog/dbglog.hpp"

#include "jsoncpp/as.hpp"

#include "./freelayer.hpp"
#include "./json.hpp"

namespace vadstena { namespace registry {

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

void build(Json::Value &content, const FreeLayer::Geodata &def)
{
    build(content["extents"], def.extents);
    content["displaySize"] = def.displaySize;
    content["label"] = def.label;
    content["geodata"] = def.geodata;
    content["style"] = def.style;
}

void build(Json::Value &content, const FreeLayer::GeodataTiles &def)
{
    build(content["lodRange"], def.lodRange);
    build(content["tileRange"], def.tileRange);

    content["metaUrl"] = def.metaUrl;
    content["geodataUrl"] = def.geodataUrl;
    content["style"] = def.style;
}

void build(Json::Value &content, const FreeLayer::MeshTiles &def)
{
    build(content["lodRange"], def.lodRange);
    build(content["tileRange"], def.tileRange);
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
    parse(def.extents, content["extents"]);
    Json::get(def.displaySize, content, "displaySize");
    Json::get(def.label, content, "label");
    Json::get(def.geodata, content, "geodata");
    Json::get(def.style, content, "style");
}

void parse(FreeLayer::GeodataTiles &def, const Json::Value &content)
{
    Json::get(def.lodRange.min, content, "lodRange", 0);
    Json::get(def.lodRange.max, content, "lodRange", 1);
    def.tileRange = tileRangeFromJson(content["tileRange"]);

    Json::get(def.metaUrl, content, "metaUrl");
    Json::get(def.geodataUrl, content, "geodataUrl");
    Json::get(def.style, content, "style");
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
    Json::StyledStreamWriter().write(out, content);
}

} } // namespace vadstena::registry
