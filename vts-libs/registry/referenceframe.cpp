/**
 * \file storage/referenceframe.cpp
 * \author Vaclav Blazek <vaclav.blazek@citationtech.net>
 */

#include <algorithm>
#include <fstream>

#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string/case_conv.hpp>

#include "dbglog/dbglog.hpp"

#include "jsoncpp/json.hpp"
#include "jsoncpp/as.hpp"

#include "../storage/error.hpp"
#include "./referenceframe.hpp"
#include "./json.hpp"
#include "./datafile.hpp"
#include "../registry.hpp"

namespace ba = boost::algorithm;

namespace vadstena { namespace registry {

constexpr char Srs::typeName[];
constexpr char ReferenceFrame::typeName[];
constexpr char Credit::typeName[];
constexpr char BoundLayer::typeName[];
constexpr char DataFile::typeName[];

TileRange tileRangeFromJson(const Json::Value &value)
{
    if (!value.isArray()) {
        LOGTHROW(err1, Json::Error)
            << "Type of tileRange is not a list.";
    }
    if (value.size() != 2) {
        LOGTHROW(err1, Json::Error)
            << "tileRange must have two elements.";
    }
    TileRange tileRange;
    Json::get(tileRange.ll(0), value[0], 0, "tileRange[0][0]");
    Json::get(tileRange.ll(1), value[0], 1, "tileRange[0][1]");
    Json::get(tileRange.ur(0), value[1], 0, "tileRange[1][0]");
    Json::get(tileRange.ur(1), value[1], 1, "tileRange[1][1]");
    return tileRange;
}

namespace {

constexpr int DEFAULT_RF_VERSION(1);

namespace v1 {

void parseIntSet(std::set<int> &set, const Json::Value &value
                 , const char *name)
{
    if (!value.isArray()) {
        LOGTHROW(err1, Json::Error)
            << "Type of " << name << " is not a list.";
    }

    for (const auto &number : value) {
        if (!number.isIntegral()) {
            LOGTHROW(err1, Json::Error)
                << "Type of " << name << " element is not an number.";
        }
        set.insert(number.asInt());
    }
}

void parse(ReferenceFrame::Model &model, const Json::Value &content)
{
    Json::get(model.physicalSrs, content, "physicalSrs");
    Json::get(model.navigationSrs, content, "navigationSrs");
    Json::get(model.publicSrs, content, "publicSrs");
}


void parse(math::Extents3 &extents, const Json::Value &content)
{
    get(extents.ll(0), content, "ll", 0);
    get(extents.ll(1), content, "ll", 1);
    get(extents.ll(2), content, "ll", 2);
    get(extents.ur(0), content, "ur", 0);
    get(extents.ur(1), content, "ur", 1);
    get(extents.ur(2), content, "ur", 2);
}

void parse(math::Extents2 &extents, const Json::Value &content)
{
    get(extents.ll(0), content, "ll", 0);
    get(extents.ll(1), content, "ll", 1);
    get(extents.ur(0), content, "ur", 0);
    get(extents.ur(1), content, "ur", 1);
}

void parse(ReferenceFrame::Division::Node::Id &id
           , const Json::Value &content)
{
    get(id.lod, content, "lod");
    get(id.x, content, "position", 0);
    get(id.y, content, "position", 1);
}

void parse(ReferenceFrame::Division::Node::Partitioning &partitioning
           , const Json::Value &content)
{
    partitioning.mode = PartitioningMode::manual;

    math::Extents2 e;

    if (content.isMember("00")) {
        parse(e, check(content["00"], Json::objectValue));
        partitioning.n00 = e;
    }

    if (content.isMember("01")) {
        parse(e, check(content["01"], Json::objectValue));
        partitioning.n01 = e;
    }

    if (content.isMember("10")) {
        parse(e, check(content["10"], Json::objectValue));
        partitioning.n10 = e;
    }

    if (content.isMember("11")) {
        parse(e, check(content["11"], Json::objectValue));
        partitioning.n11 = e;
    }
}

void parse(ReferenceFrame::Division::Node &node, const Json::Value &content)
{
    const auto &id(content["id"]);
    if (!id.isObject()) {
        LOGTHROW(err1, Json::Error)
            << "Type of node[id] is not an object.";
    }
    parse(node.id, id);

    const auto &partitioning(content["partitioning"]);
    if (partitioning.isObject()) {
        // object
        parse(node.partitioning, partitioning);
    } else {
        // just mode
        std::string s;
        node.partitioning.mode
            = boost::lexical_cast<PartitioningMode>
            (Json::get(s, content, "partitioning"));
    }

    if (content.isMember("srs")) {
        Json::get(node.srs, content, "srs");
    }

    if (content.isMember("extents")) {
        const auto &extents(content["extents"]);
        if (!extents.isObject()) {
            LOGTHROW(err1, Json::Error)
                << "Type of node(" << node.id
                << ")[extents] is not an object.";
        }
        parse(node.extents, extents);
    }

    if (content.isMember("externalTexture")) {
        Json::get(node.externalTexture, content, "externalTexture");
    }
}

void parse(ReferenceFrame::Division &division, const Json::Value &content)
{
    const auto &extents(content["extents"]);
    if (!extents.isObject()) {
        LOGTHROW(err1, Json::Error)
            << "Type of division[extents] is not an object.";
    }
    parse(division.extents, extents);
    Json::get(division.heightRange.min, content, "heightRange", 0);
    Json::get(division.heightRange.max, content, "heightRange", 1);

    const auto &nodes(content["nodes"]);
    if (!nodes.isArray()) {
        LOGTHROW(err1, Json::Error)
            << "Type of division[nodes] is not a list.";
    }

    for (const auto &element : nodes) {
        ReferenceFrame::Division::Node node;
        parse(node, element);
        division.nodes.insert
            (ReferenceFrame::Division::Node::map::value_type
             (node.id, node));
    }
}

void parse(ReferenceFrame &rf, const Json::Value &content)
{
    Json::get(rf.id, content, "id");
    Json::get(rf.description, content, "description");

    const auto &model(content["model"]);
    if (!model.isObject()) {
        LOGTHROW(err1, Json::Error)
            << "Type of referenceframe[model] is not an object.";
    }
    parse(rf.model, model);

    const auto &division(content["division"]);
    if (!division.isObject()) {
        LOGTHROW(err1, Json::Error)
            << "Type of referenceframe[division] is not an object.";
    }
    parse(rf.division, division);

    const auto &parameters(content["parameters"]);
    if (!parameters.isObject()) {
        LOGTHROW(err1, Json::Error)
            << "Type of referenceframe[parameters] is not an object.";
    }
    Json::get(rf.metaBinaryOrder, parameters, "metaBinaryOrder");
}

} // namespace v1


ReferenceFrame::Division::Node::Id
child(const ReferenceFrame::Division::Node::Id &id, bool x, bool y)
{
    return ReferenceFrame::Division::Node::Id
        (id.lod + 1, id.x + x, id.y + y);
}

/** Checks whether node is under root.
 */
bool isUnder(const ReferenceFrame::Division::Node::Id &root
             , const ReferenceFrame::Division::Node::Id &node)
{
    if (node.lod <= root.lod) {
        // not bellow LOD-wise
        return false;
    }
    auto diff(node.lod - root.lod);

    // check x and y transformed to root's LOD
    if (root.x != (node.x >> diff)) { return false; }
    if (root.y != (node.y >> diff)) { return false; }

    // node is in root's subtree
    return true;
}

void sanitize(const boost::filesystem::path &path
              , ReferenceFrame &rf)
{
    typedef ReferenceFrame::Division::Node Node;

    std::vector<Node::Id> invalid;

    auto &div(rf.division);
    for (const auto &item : div.nodes) {
        const auto &id(item.first);
        const auto &part(item.second.partitioning);

        auto c00(child(id, 0, 0));
        auto c01(child(id, 0, 1));
        auto c10(child(id, 1, 0));
        auto c11(child(id, 1, 1));

        // find all node children
        auto *n00(div.find(c00, std::nothrow));
        auto *n01(div.find(c01, std::nothrow));
        auto *n10(div.find(c10, std::nothrow));
        auto *n11(div.find(c11, std::nothrow));

        if (part.mode != PartitioningMode::manual) {
            // bisection of nothing at all -- there must be mothing inside
            // partitioning and no physical node
            if (part.n00 || n00 || part.n01 || n01
                || part.n10 || n10 || part.n11 || n11)
            {
                LOGTHROW(err2, storage::FormatError)
                    << "Unable to parse reference frame file " << path
                    << ": reference frame <" << rf.id
                    << "> has invalid partitioning of node " << id
                    << " (either child node or paritioning definition exists "
                    "for non-manual node).";
            }
            continue;
        }

        // manual partitioning -> check that at least child node exists, every
        // child node from partitioning exists and remember invalid children

        int left(4);
        auto check([&](const Node::Id &child
                       , const boost::optional<math::Extents2> &extents
                       , Node *node)
            mutable
        {
            if (bool(extents) != bool(node)) {
                LOGTHROW(err2, storage::FormatError)
                    << "Unable to parse reference frame file " << path
                    << ": reference frame <" << rf.id
                    << "> has invalid partitioning of node " << id
                    << " (mismatch in definition of child node "
                    << child << ").";
            }

            if (!extents) {
                invalid.push_back(child);
                --left;
            } else {
                // create manual partition information
                node->constraints
                    = boost::in_place(*extents, item.second.srs);
            }
        });

        check(c00, part.n00, n00);
        check(c01, part.n01, n01);
        check(c10, part.n10, n10);
        check(c11, part.n11, n11);

        if (!left) {
            LOGTHROW(err2, storage::FormatError)
                << "Unable to parse reference frame file " << path
                << ": reference frame <" << rf.id
                << "> has invalid partitioning of node " << id
                << " (no child node defined).";
        }
    }

    // insert invalid nodes
    for (const auto &id : invalid) {
        LOG(debug) << rf.id << ": generating invalid node " << id << ".";
        div.nodes.insert(Node::map::value_type
                         (id, Node(id, PartitioningMode::none)));
    }
}

void parse(const boost::filesystem::path &path
           , ReferenceFrame::dict &rfs, const Json::Value &content)
{
    for (const auto &element : Json::check(content, Json::arrayValue)) {
        ReferenceFrame rf;
        try {
            int version(0);
            Json::get(version, element, "version");

            switch (version) {
            case 1:
                v1::parse(rf, element);
                break;

            default:
                LOGTHROW(err1, storage::FormatError)
                    << "Invalid reference frame file format: "
                    "unsupported version "
                    << version << ".";
            }
        } catch (const Json::Error &e) {
            LOGTHROW(err1, storage::FormatError)
                << "Invalid reference frame file format (" << e.what()
                << ").";
        }

        sanitize(path, rf);
        rfs.add(rf);
    }
}

void build(Json::Value &content, const ReferenceFrame::Model &model)
{
    content = Json::objectValue;
    content["physicalSrs"] = model.physicalSrs;
    content["navigationSrs"] = model.navigationSrs;
    content["publicSrs"] = model.publicSrs;
}

void build(Json::Value &content, const math::Extents3 &extents)
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

void build(Json::Value &content, const math::Extents2 &extents)
{
    content = Json::objectValue;

    auto &ll(content["ll"] = Json::arrayValue);
    ll.append(extents.ll(0));
    ll.append(extents.ll(1));

    auto &ur(content["ur"] = Json::arrayValue);
    ur.append(extents.ur(0));
    ur.append(extents.ur(1));
}

void build(Json::Value &content, const ReferenceFrame::Division::Node::Id &id)
{
    content = Json::objectValue;
    content["lod"] = id.lod;
    auto &position(content["position"] = Json::arrayValue);
    position.append(id.x);
    position.append(id.y);
}

void build(Json::Value &content
           , const ReferenceFrame::Division::Node::Partitioning &partitioning)
{
    switch (partitioning.mode) {
    case PartitioningMode::none:
    case PartitioningMode::bisection:
        content = boost::lexical_cast<std::string>(partitioning.mode);
        return;

    default: break;
    }

    // manual
    content = Json::objectValue;

    if (partitioning.n00) { build(content["00"], *partitioning.n00); }
    if (partitioning.n01) { build(content["01"], *partitioning.n01); }
    if (partitioning.n10) { build(content["10"], *partitioning.n10); }
    if (partitioning.n11) { build(content["11"], *partitioning.n11); }
}

void build(Json::Value &content, const ReferenceFrame::Division::Node &node)
{
    content = Json::objectValue;

    build(content["id"], node.id);
    build(content["partitioning"], node.partitioning);

    if (node.partitioning.mode != PartitioningMode::none) {
        content["srs"] = node.srs;
        build(content["extents"], node.extents);
    }

    // write only when true
    if (node.externalTexture) {
        content["externalTexture"] = node.externalTexture;
    }
}

void build(Json::Value &content, const ReferenceFrame::Division &division)
{
    content = Json::objectValue;

    build(content["extents"], division.extents);

    auto &heightRange(content["heightRange"] = Json::arrayValue);
    heightRange.append(division.heightRange.min);
    heightRange.append(division.heightRange.max);

    auto &nodes(content["nodes"]);
    for (const auto &node : division.nodes) {
        // only valid nodes are serialized
        if (node.second.valid()) {
            build(nodes.append(Json::nullValue), node.second);
        }
    }
}

void build(Json::Value &content, const ReferenceFrame &rf)
{
    content = Json::objectValue;
    content["version"] = DEFAULT_RF_VERSION;

    content["id"] = rf.id;
    content["description"] = rf.description;
    build(content["model"], rf.model);
    build(content["division"], rf.division);

    auto &parameters(content["parameters"] = Json::objectValue);
    parameters["metaBinaryOrder"] = rf.metaBinaryOrder;
}

void build(Json::Value &content, const ReferenceFrame::dict &rfs)
{
    content = Json::arrayValue;

    for (const auto &rf : rfs) {
        build(content.append(Json::nullValue), rf.second);
    }
}

GeoidGrid parseGeoidGrid(const Json::Value &content)
{
    if (!content.isObject()) {
        LOGTHROW(err1, Json::Error)
            << "GeoidGrid is not an object.";
    }

    GeoidGrid gg;
    v1::parse(gg.extents, content["extents"]);

    Json::get(gg.valueRange.min, content, "valueRange", 0);
    Json::get(gg.valueRange.max, content, "valueRange", 1);

    Json::get(gg.definition, content, "definition");

    std::string s;
    gg.srsDefEllps = { Json::get(s, content, "srsDefEllps")
                       , geo::SrsDefinition::Type::proj4 };
    return gg;
}

Periodicity parsePeriodicity(const Json::Value &content)
{
    if (!content.isObject()) {
        LOGTHROW(err1, Json::Error)
            << "Periodicity is not an object.";
    }

    Periodicity p;

    std::string s;
    p.type = boost::lexical_cast<Periodicity::Type>
        (Json::get(s, content, "type"));
    Json::get(p.period, content, "period");

    return p;
}

void parse(Srs &srs, const Json::Value &content)
{
    Json::get(srs.comment, content, "comment");

    // just mode
    std::string s;
    srs.type = boost::lexical_cast<Srs::Type>(Json::get(s, content, "type"));
    srs.srsDef = { Json::get(s, content, "srsDef")
                   , geo::SrsDefinition::Type::proj4};
    if (content.isMember("geoidGrid")) {
        srs.geoidGrid = parseGeoidGrid(content["geoidGrid"]);
    }

    if (content.isMember("periodicity")) {
        srs.periodicity = parsePeriodicity(content["periodicity"]);
    }

    // parse modifiers
    if (content.isMember("srsModifiers")) {
        for (const auto &modifier
                 : Json::check(content["srsModifiers"], Json::arrayValue))
        {
            s = Json::as<std::string>(modifier, "srsModifiers");
            if (s == "adjustVertical") {
                srs.srsModifiers |= Srs::Modifiers::adjustVertical;
            } else {
                LOGTHROW(err1, Json::Error)
                    << "Invalid value " << s << " in srsModifiers.";
            }
        }
    }
}

void parse(Srs::dict &srs, const Json::Value &content)
{
    for (const auto &id : Json::check(content, Json::objectValue)
             .getMemberNames())
    {
        try {
            Srs s;
            parse(s, Json::check(content[id], Json::objectValue));
            srs.set(id, s);
        } catch (const Json::Error &e) {
            LOGTHROW(err1, storage::FormatError)
                << "Invalid srs file format (" << e.what()
                << ").";
        }
    }
}

void build(Json::Value &content, const Srs &srs
           , bool flatGeoidPath = false)
{
    content = Json::objectValue;
    content["comment"] = srs.comment;
    content["type"] = boost::lexical_cast<std::string>(srs.type);
    content["srsDef"] = srs.srsDef.as(geo::SrsDefinition::Type::proj4).srs;

    if (srs.geoidGrid) {
        const auto &gg(*srs.geoidGrid);
        auto &jgg(content["geoidGrid"] = Json::objectValue);

        build(jgg["extents"], gg.extents);

        auto &valueRange(jgg["valueRange"] = Json::arrayValue);
        valueRange.append(gg.valueRange.min);
        valueRange.append(gg.valueRange.max);

        if (flatGeoidPath) {
            // take only filename
            jgg["definition"] = boost::filesystem::path
                (gg.definition).filename().string();
        } else {
            jgg["definition"] = gg.definition;
        }
        jgg["srsDefEllps"]
            = gg.srsDefEllps.as(geo::SrsDefinition::Type::proj4).srs;
    }

    if (srs.periodicity) {
        const auto &p(*srs.periodicity);
        auto &jp(content["periodicity"] = Json::objectValue);

        jp["type"] = boost::lexical_cast<std::string>(p.type);
        jp["period"] = p.period;
    }

    if (srs.srsModifiers) {
        auto &srsModifiers(content["srsModifiers"] =  Json::arrayValue);
        if (srs.srsModifiers & Srs::Modifiers::adjustVertical) {
            srsModifiers.append("adjustVertical");
        }
    }
}

void build(Json::Value &content, const Srs::dict &srs
           , bool flatGeoidPath = false)
{
    content = Json::objectValue;

    for (const auto &s : srs) {
        build(content[s.first], s.second,  flatGeoidPath);
    }
}

void parse(Credit &c, const Json::Value &content)
{
    Json::get(c.numericId, content, "id");

    Json::get(c.notice, content, "notice");
    if (content.isMember("url")) {
        c.url = std::string();
        Json::get(*c.url, content, "url");
    }
    if (content.isMember("copyrighted")) {
        get(c.copyrighted, content, "copyrighted");
    } else {
        c.copyrighted = true;
    }
}

void parse(Credit::dict &credits, const Json::Value &content)
{
    for (const auto &id : Json::check(content, Json::objectValue)
             .getMemberNames())
    {
        try {
            Credit c;
            c.id = id;
            parse(c, Json::check(content[id], Json::objectValue));
            credits.set(id, c);
        } catch (const Json::Error &e) {
            LOGTHROW(err1, storage::FormatError)
                << "Invalid credits file format (" << e.what()
                << ").";
        }
    }
}

void build(Json::Value &content, const Credit &c)
{
    content = Json::objectValue;
    content["id"] = c.numericId;
    content["notice"] = c.notice;
    if (c.url) { content["url"] = *c.url; }
    if (!c.copyrighted) { content["copyrighted"] = c.copyrighted; }
}

void build(Json::Value &content, const Credit::dict &credits)
{
    content = Json::objectValue;

    for (const auto &c : credits) {
        build(content[c.first], c.second);
    }
}

void parse(BoundLayer &bl, const Json::Value &content)
{
    if (content.isMember("id")) {
        Json::get(bl.numericId, content, "id");
    }

    std::string s;
    bl.type = boost::lexical_cast<BoundLayer::Type>
        (Json::get(s, content, "type"));

    Json::get(bl.url, content, "url");
    if (content.isMember("maskUrl")) {
        bl.maskUrl = boost::in_place();
        Json::get(*bl.maskUrl, content, "maskUrl");
    }
    if (content.isMember("metaUrl")) {
        bl.metaUrl = boost::in_place();
        Json::get(*bl.metaUrl, content, "metaUrl");
    }
    Json::get(bl.lodRange.min, content, "lodRange", 0);
    Json::get(bl.lodRange.max, content, "lodRange", 1);

    bl.tileRange = tileRangeFromJson(content["tileRange"]);

    const auto &credits(content["credits"]);
    if (credits.isArray()) {
        for (const auto &element : credits) {
            bl.credits.set(element.asString(), boost::none);
        }
    } else if (credits.isObject()) {
        for (const auto &id : Json::check(credits, Json::objectValue)
                 .getMemberNames())
        {
            const auto &element(Json::check(credits[id], Json::objectValue));
            if (element.empty()) {
                bl.credits.set(id, boost::none);
            } else {
                Credit c;
                c.id = id;
                parse(c, element);
                bl.credits.set(id, c);
            }
        }
    } else {
        LOGTHROW(err1, Json::Error)
            << "Type of boundLayer[credits] is not a list nor an object.";
    }

    if (content.isMember("availability")) {
        const auto &availability(content["availability"]);

        if (!availability.isObject()) {
            LOGTHROW(err1, Json::Error)
                << "Type of boundLayer[availability] is not an object.";
        }

        bl.availability = boost::in_place();
        auto &bla(*bl.availability);

        bla.type = boost::lexical_cast<BoundLayer::Availability::Type>
            (Json::get(s, availability, "type"));

        switch (bla.type) {
        case BoundLayer::Availability::Type::negativeType:
            Json::get(bla.mime, availability, "mime");
            break;

        case BoundLayer::Availability::Type::negativeCode:
            v1::parseIntSet(bla.codes, availability["codes"]
                            , "boundLayer[availability[codes]]");
            break;

        case BoundLayer::Availability::Type::negativeSize:
            Json::get(bla.size, availability, "size");
            break;
        }
    }

    // isTransparent is optional
    if (content.isMember("isTransparent")) {
        Json::get(bl.isTransparent, content, "isTransparent");
    }
}

void parse(BoundLayer::dict &bls, const Json::Value &content)
{
    for (const auto &id : Json::check(content, Json::objectValue)
             .getMemberNames())
    {
        try {
            BoundLayer bl;
            bl.id = id;
            const auto &value(content[id]);
            if (value.isString()) {
                // special case: external url
                bl.type = BoundLayer::Type::external;
                bl.url = value.asString();
            } else {
                parse(bl, Json::check(value, Json::objectValue));
            }
            bls.set(id, bl);
        } catch (const Json::Error &e) {
            LOGTHROW(err1, storage::FormatError)
                << "Invalid bound layers file format (" << e.what()
                << ").";
        }
    }
}

void build(Json::Value &value, const Credits &credits
           , bool inlineCredits = true)
{
    bool hasInlineCredits(inlineCredits && std::accumulate
                          (credits.begin(), credits.end()
                           , 0, [](int v, const Credits::value_type &item)
                           {
                               return v + bool(item.second);
                           }));

    if (hasInlineCredits) {
        value = Json::objectValue;
        for (const auto &item : credits) {
            // add empty dictionary
            auto &credit(value[item.first] = Json::objectValue);
            if (item.second) {
                // build content if there is some
                build(credit, *item.second);
            }
        }
        return;
    }

    // can be represented as a vector of identifiers
    value = Json::arrayValue;
    for (const auto &item : credits) {
        value.append(item.first);
    };
}

void build(Json::Value &content, const BoundLayer &bl
           , bool inlineCredits = true)
{
    content = Json::objectValue;
    if (bl.numericId > 0) {
        // numeric id is optional
        content["id"] = bl.numericId;
    }
    content["type"] = boost::lexical_cast<std::string>(bl.type);
    content["url"] = bl.url;
    if (bl.maskUrl) { content["maskUrl"] = *bl.maskUrl; }
    if (bl.metaUrl) { content["metaUrl"] = *bl.metaUrl; }

    auto &lodRange(content["lodRange"] = Json::arrayValue);
    lodRange.append(bl.lodRange.min);
    lodRange.append(bl.lodRange.max);

    auto &tileRange(content["tileRange"] = Json::arrayValue);
    auto &tileRangeMin(tileRange.append(Json::arrayValue));
    tileRangeMin.append(bl.tileRange.ll(0));
    tileRangeMin.append(bl.tileRange.ll(1));
    auto &tileRangeMax(tileRange.append(Json::arrayValue));
    tileRangeMax.append(bl.tileRange.ur(0));
    tileRangeMax.append(bl.tileRange.ur(1));

    build(content["credits"], bl.credits, inlineCredits);

    if (bl.availability) {
        auto &availability(content["availability"] = Json::objectValue);
        const auto &bla(*bl.availability);
        availability["type"] = boost::lexical_cast<std::string>(bla.type);

        switch (bla.type) {
        case BoundLayer::Availability::Type::negativeType:
            availability["mime"] = bla.mime;
            break;

        case BoundLayer::Availability::Type::negativeCode:
            {
                auto &codes(availability["codes"] = Json::arrayValue);
                for (auto code : bla.codes) { codes.append(code); }
            }
            break;

        case BoundLayer::Availability::Type::negativeSize:
            availability["size"] = bla.size;
            break;
        }
    }

    // isTransparent is optional, defaults to false
    if (bl.isTransparent) {
        content["isTransparent"] = bl.isTransparent;
    }
}

void build(Json::Value &content, const BoundLayer::dict &bls
           , bool inlineCredits = true)
{
    content = Json::objectValue;

    for (const auto &item : bls) {
        const auto &id(item.first);
        const auto &bl(item.second);

        if (bl.type == BoundLayer::Type::external) {
            // external bound layer: just URL
            content[id] = bl.url;
        } else {
            // regular bound layer
            build(content[id], bl, inlineCredits);
        }
    }
}

} // namesapce

ReferenceFrame::dict loadReferenceFrames(std::istream &in
                                         , const boost::filesystem::path &path)
{
    // load json
    Json::Value content;
    Json::Reader reader;
    if (!reader.parse(in, content)) {
        LOGTHROW(err2, storage::FormatError)
            << "Unable to parse reference frame file " << path << ": "
            << reader.getFormattedErrorMessages() << ".";
    }

    ReferenceFrame::dict rfs;
    parse(path, rfs, content);
    return rfs;
}

ReferenceFrame::dict loadReferenceFrames(const boost::filesystem::path &path)
{
    LOG(info1) << "Loading reference frame file from " << path  << ".";
    std::ifstream f;
    f.exceptions(std::ios::badbit | std::ios::failbit);
    try {
        f.open(path.string(), std::ios_base::in);
    } catch (const std::exception &e) {
        LOGTHROW(err1, storage::IOError)
            << "Unable to load reference frame file file " << path << ".";
    }
    auto rfs(loadReferenceFrames(f, path));
    f.close();
    return rfs;
}

void saveReferenceFrames(std::ostream &out
                         , const ReferenceFrame::dict &rfs)
{
    Json::Value content;
    build(content, rfs);
    out.precision(15);
    Json::StyledStreamWriter().write(out, content);
}

void saveReferenceFrames(const boost::filesystem::path &path
                         , const ReferenceFrame::dict &rfs)
{
    LOG(info1) << "Saving reference frame file file to " << path  << ".";
    std::ofstream f;
    try {
        f.exceptions(std::ios::badbit | std::ios::failbit);
        f.open(path.string(), std::ios_base::out);
    } catch (const std::exception &e) {
        LOGTHROW(err1, storage::IOError)
            << "Unable to save reference frame file file " << path << ".";
    }
    saveReferenceFrames(f, rfs);
    f.close();
}

Srs::dict loadSrs(std::istream &in)
{
    // load json
    Json::Value content;
    Json::Reader reader;
    if (!reader.parse(in, content)) {
        LOGTHROW(err2, storage::FormatError)
            << "Unable to parse srs file: "
            << reader.getFormattedErrorMessages() << ".";
    }

    Srs::dict srs;
    parse(srs, content);
    return srs;
}

Srs::dict loadSrs(const boost::filesystem::path &path)
{
    LOG(info1) << "Loading srs file from " << path  << ".";
    std::ifstream f;
    f.exceptions(std::ios::badbit | std::ios::failbit);
    try {
        f.open(path.string(), std::ios_base::in);
    } catch (const std::exception &e) {
        LOGTHROW(err1, storage::IOError)
            << "Unable to load srs file file " << path << ".";
    }
    auto srs(loadSrs(f));
    f.close();
    return srs;
}

void saveSrs(std::ostream &out, const Srs::dict &srs)
{
    Json::Value content;
    build(content, srs);
    out.precision(15);
    Json::StyledStreamWriter().write(out, content);
}

void saveSrs(const boost::filesystem::path &path
             , const Srs::dict &srs)
{
    LOG(info1) << "Saving srs file file to " << path  << ".";
    std::ofstream f;
    try {
        f.exceptions(std::ios::badbit | std::ios::failbit);
        f.open(path.string(), std::ios_base::out);
    } catch (const std::exception &e) {
        LOGTHROW(err1, storage::IOError)
            << "Unable to save srs file file " << path << ".";
    }
    saveSrs(f, srs);
    f.close();
}

BoundLayer::dict loadBoundLayers(std::istream &in)
{
    // load json
    Json::Value content;
    Json::Reader reader;
    if (!reader.parse(in, content)) {
        LOGTHROW(err2, storage::FormatError)
            << "Unable to parse bound layers file: "
            << reader.getFormattedErrorMessages() << ".";
    }

    BoundLayer::dict boundLayers;
    parse(boundLayers, content);
    return boundLayers;
}

BoundLayer::dict loadBoundLayers(const boost::filesystem::path &path)
{
    LOG(info1) << "Loading bound layers file from " << path  << ".";
    std::ifstream f;
    f.exceptions(std::ios::badbit | std::ios::failbit);
    try {
        f.open(path.string(), std::ios_base::in);
    } catch (const std::exception &e) {
        LOGTHROW(err1, storage::IOError)
            << "Unable to load bound layer file file " << path << ".";
    }
    auto boundLayers(loadBoundLayers(f));
    f.close();
    return boundLayers;
}

void saveBoundLayers(std::ostream &out, const BoundLayer::dict &boundLayers)
{
    Json::Value content;
    build(content, boundLayers);
    out.precision(15);
    Json::StyledStreamWriter().write(out, content);
}

void saveBoundLayers(const boost::filesystem::path &path
                     , const BoundLayer::dict &boundLayers)
{
    LOG(info1) << "Saving bound layers file file to " << path  << ".";
    std::ofstream f;
    try {
        f.exceptions(std::ios::badbit | std::ios::failbit);
        f.open(path.string(), std::ios_base::out);
    } catch (const std::exception &e) {
        LOGTHROW(err1, storage::IOError)
            << "Unable to save bound Layers file file " << path << ".";
    }
    saveBoundLayers(f, boundLayers);
    f.close();
}

void saveBoundLayer(std::ostream &out, const BoundLayer &boundLayer)
{
    Json::Value content;
    build(content, boundLayer);
    out.precision(15);
    Json::StyledStreamWriter().write(out, content);
}

BoundLayer loadBoundLayer(std::istream &in
                          , const boost::filesystem::path &path)
{
    // load json
    Json::Value content;
    Json::Reader reader;
    if (!reader.parse(in, content)) {
        LOGTHROW(err2, storage::FormatError)
            << "Unable to parse bound layer file " << path << ": "
            << reader.getFormattedErrorMessages() << ".";
    }

    BoundLayer boundLayer;
    parse(boundLayer, content);
    return boundLayer;
}

Credit::dict loadCredits(std::istream &in)
{
    // load json
    Json::Value content;
    Json::Reader reader;
    if (!reader.parse(in, content)) {
        LOGTHROW(err2, storage::FormatError)
            << "Unable to parse credits file: "
            << reader.getFormattedErrorMessages() << ".";
    }

    Credit::dict credits;
    parse(credits, content);
    return credits;
}

Credit::dict loadCredits(const boost::filesystem::path &path)
{
    LOG(info1) << "Loading credits file from " << path  << ".";
    std::ifstream f;
    f.exceptions(std::ios::badbit | std::ios::failbit);
    try {
        f.open(path.string(), std::ios_base::in);
    } catch (const std::exception &e) {
        LOGTHROW(err1, storage::IOError)
            << "Unable to load credits from file " << path << ".";
    }
    auto credits(loadCredits(f));
    f.close();
    return credits;
}

void saveCredits(std::ostream &out, const Credit::dict &credits)
{
    Json::Value content;
    build(content, credits);
    out.precision(15);
    Json::StyledStreamWriter().write(out, content);
}

void saveCredits(const boost::filesystem::path &path
                     , const Credit::dict &credits)
{
    LOG(info1) << "Saving credits to file " << path  << ".";
    std::ofstream f;
    try {
        f.exceptions(std::ios::badbit | std::ios::failbit);
        f.open(path.string(), std::ios_base::out);
    } catch (const std::exception &e) {
        LOGTHROW(err1, storage::IOError)
            << "Unable to save credits to file " << path << ".";
    }
    saveCredits(f, credits);
    f.close();
}

void saveCredits(std::ostream &out, const Credits &credits
                 , bool inlineCredits)
{
    Json::Value content;
    build(content, credits, inlineCredits);
    out.precision(15);
    Json::StyledStreamWriter().write(out, content);
}

const ReferenceFrame::Division::Node*
ReferenceFrame::Division::find(const Node::Id &id, std::nothrow_t) const
{
    auto fnodes(nodes.find(id));
    if (fnodes == nodes.end()) { return nullptr; }
    return &fnodes->second;
}

ReferenceFrame::Division::Node*
ReferenceFrame::Division::find(const Node::Id &id, std::nothrow_t)
{
    auto fnodes(nodes.find(id));
    if (fnodes == nodes.end()) { return nullptr; }
    return &fnodes->second;
}

const ReferenceFrame::Division::Node&
ReferenceFrame::Division::find(const Node::Id &id) const
{
    const auto *node(find(id, std::nothrow));
    if (!node) {
        LOGTHROW(err1, storage::KeyError)
            << "No node <" << id << "> in division tree.";
    }
    return *node;
}

std::set<std::string> ReferenceFrame::Division::srsList() const
{
    std::set<std::string> list;
    for (const auto &item : nodes) {
        if (item.second.valid()) {
            list.insert(item.second.srs);
        }
    }
    return list;
}

math::Extents2 ReferenceFrame::rootExtents() const
{
    return division.find({}).extents;
}

math::Size2f ReferenceFrame::tileSize(Lod lod) const
{
    auto ts(size(rootExtents()));
    ts.width /= (1 << lod);
    ts.height /= (1 << lod);
    return ts;
}

std::string ReferenceFrame::rootSrs() const
{
    return division.find({}).srs;
}

Json::Value asJson(const ReferenceFrame &rf)
{
    Json::Value content;
    build(content, rf);
    return content;
}

Json::Value asJson(const Srs::dict &srs, bool flatGeoidPath)
{
    Json::Value content;
    build(content, srs, flatGeoidPath);
    return content;
}

Json::Value asJson(const Credit::dict &credits)
{
    Json::Value content;
    build(content, credits);
    return content;
}

void fromJson(Credit::dict &credits, const Json::Value &value)
{
    parse(credits, value);
}

Json::Value asJson(const BoundLayer::dict &boundLayers
                   , bool inlineCredits)
{
    Json::Value content;
    build(content, boundLayers, inlineCredits);
    return content;
}

void fromJson(BoundLayer::dict &boundLayers, const Json::Value &value)
{
    parse(boundLayers, value);
}

Json::Value asJson(const Position &position)
{
    Json::Value value(Json::arrayValue);
    value.append(boost::lexical_cast<std::string>(position.type));
    value.append(position.position(0));
    value.append(position.position(1));
    value.append(boost::lexical_cast<std::string>(position.heightMode));
    value.append(position.position(2));
    value.append(position.orientation(0));
    value.append(position.orientation(1));
    value.append(position.orientation(2));
    value.append(position.verticalExtent);
    value.append(position.verticalFov);
    return value;
}

Position positionFromJson(const Json::Value &value)
{
    Position p;

    if (!value.isArray()) {
        LOGTHROW(err1, Json::Error)
            << "Type of position is not a list.";
    }

    p.type = boost::lexical_cast<registry::Position::Type>
        (Json::as<std::string>(value[0]));
    p.position(0) = Json::as<double>(value[1]);
    p.position(1) = Json::as<double>(value[2]);
    p.heightMode = boost::lexical_cast<registry::Position::HeightMode>
        (Json::as<std::string>(value[3]));
    p.position(2) = Json::as<double>(value[4]);

    p.orientation(0) = Json::as<double>(value[5]);
    p.orientation(1) = Json::as<double>(value[6]);
    p.orientation(2) = Json::as<double>(value[7]);

    p.verticalExtent = Json::as<double>(value[8]);
    p.verticalFov = Json::as<double>(value[9]);

    return p;
}

Srs::dict listSrs(const ReferenceFrame &referenceFrame)
{
    Srs::dict srs;

    auto add([&](const std::string &id)
    {
        srs.set(id, Registry::srs(id));
    });

    add(referenceFrame.model.physicalSrs);
    add(referenceFrame.model.navigationSrs);
    add(referenceFrame.model.publicSrs);

    for (const auto &node : referenceFrame.division.nodes) {
        if (node.second.valid()) {
            add(node.second.srs);
        }
    }

    return srs;
}

Credit::dict creditsAsDict(const StringIdSet &credits)
{
    Credit::dict c;
    for (const auto &id : credits) {
        c.add(Registry::credit(id, std::nothrow));
    };
    return c;
}

Credit::dict creditsAsDict(const Credits &credits)
{
    Credit::dict c;
    for (const auto &item : credits) {
        if (item.second) {
            // value
            c.add(*item.second);
        } else {
            // just id, resolve
            c.add(Registry::credit(item.first, std::nothrow));
        }
    };
    return c;
}

Credit::dict creditsAsDict(const IdSet &credits)
{
    Credit::dict c;
    for (const auto &id : credits) {
        c.add(Registry::credit(id, std::nothrow));
    };
    return c;
}

BoundLayer::dict boundLayersAsDict(const StringIdSet &boundLayers)
{
    BoundLayer::dict b;
    for (const auto &id : boundLayers) {
        b.add(Registry::boundLayer(id, std::nothrow));
    };
    return b;
}

BoundLayer::dict boundLayersAsDict(const IdSet &boundLayers)
{
    BoundLayer::dict b;
    for (const auto &id : boundLayers) {
        b.add(Registry::boundLayer(id, std::nothrow));
    };
    return b;
}

const ReferenceFrame::Division::Node&
ReferenceFrame::Division::findSubtreeRoot(const Node::Id &nodeId) const
{
    LOG(debug) << "Finding subtree root for node " << nodeId;
    const Node *candidate(nullptr);
    for (const auto &item : nodes) {
        const auto &treeRoot(item.first);

        if (candidate && (treeRoot.lod <= candidate->id.lod )) {
            // this root would be above or at already found candidate root ->
            // impossible
            continue;
        }

        if (treeRoot.lod > nodeId.lod) {
            // root cannot be bellow node
            continue;
        }

        if (treeRoot == nodeId) {
            // exact match -> standing at the root
            return item.second;
        }

        // node bellow possible root -> check if it can be a candidate
        auto diff(nodeId.lod - treeRoot.lod);
        Node::Id::index_type x(nodeId.x >> diff), y(nodeId.y >> diff);
        if ((treeRoot.x == x) && (treeRoot.y == y)) {
            // match -> nodeId is bellow treeRoot -> remember
            candidate = &item.second;
        }
    }

    if (!candidate) {
        LOGTHROW(err1, storage::KeyError)
            << "Node <" << nodeId << "> has no root in this reference frame.";
    }

    return *candidate;
}

Json::Value asJson(const View &view, BoundLayer::dict &boundLayers)
{
    Json::Value nv;

    if (view.description) {
        nv["description"] = *view.description;
    }
    auto &surfaces(nv["surfaces"] = Json::objectValue);

    for (const auto &surfaceItem : view.surfaces) {
        auto &surface(surfaces[surfaceItem.first] = Json::arrayValue);
        for (const auto &blp : surfaceItem.second) {
            if (blp.isComplex()) {
                auto &p(surface.append(Json::objectValue));
                p["id"] = blp.id;
                if (blp.alpha) { p["alpha"] = *blp.alpha; }
            } else {
                surface.append(blp.id);
            }
            boundLayers.add(registry::Registry::boundLayer
                            (blp.id, std::nothrow));
        }
    }

    auto &fls(nv["freeLayers"] = Json::arrayValue);
    for (const auto &fl : view.freeLayers) { fls.append(fl); }

    return nv;
}

View viewFromJson(const Json::Value &value)
{
    if (!value.isObject()) {
        LOGTHROW(err1, Json::Error)
            << "Type of view is not an object.";
    }

    View view;

    if (value.isMember("description")) {
        view.description = boost::in_place();
        Json::get(*view.description, value, "description");
    }

    const auto &surfaces(value["surfaces"]);
    if (!surfaces.isObject()) {
        LOGTHROW(err1, Json::Error)
            << "Type of namedView[surfaces] is not an object.";
    }

    for (const auto &sId : surfaces.getMemberNames()) {
        const auto &surface(surfaces[sId]);
        if (!surface.isArray()) {
            LOGTHROW(err1, Json::Error)
                << "Type of namedView[surfaces][" << sId
                << "] is not a list.";
        }

        auto &blpList(view.surfaces[sId]);

        for (const auto &blp : surface) {
            if (blp.isObject()) {
                blpList.emplace_back();
                auto &item(blpList.back());
                Json::get(item.id, blp, "id");
                if (blp.isMember("alpha")) {
                    item.alpha = double();
                    Json::get(*item.alpha, blp, "alpha");
                }
            } else if (blp.isString()) {
                blpList.push_back(blp.asString());
            } else {
                LOGTHROW(err1, Json::Error)
                    << "Type of namedView[surfaces][" << sId
                    << "] item must be either string or object.";
            }
        }
    }

    const auto &freeLayers(value["freeLayers"]);
    if (!freeLayers.isNull()) {
        if (!freeLayers.isArray()) {
            LOGTHROW(err1, Json::Error)
                << "Type of view[freeLayers] member is not a list.";
        }

        for (const auto &fl : freeLayers) {
            if (!fl.isString()) {
                LOGTHROW(err1, Json::Error)
                    << "Type of view[freeLayers] member is not a string.";
            }
            view.freeLayers.insert(fl.asString());
        }
    }

    return view;
}

Json::Value asJson(const Roi &roi)
{
    Json::Value v(Json::objectValue);
    v["exploreUrl"] = roi.exploreUrl;
    v["roiUrl"] = roi.roiUrl;
    v["thumbnailUrl"] = roi.thumbnailUrl;
    return v;
}

Json::Value asJson(const Roi::list &rois)
{
    Json::Value a(Json::arrayValue);
    for (const auto roi : rois) { a.append(asJson(roi)); }
    return a;
}

Roi::list roisFromJson(const Json::Value &value)
{
    if (!value.isArray()) {
        LOGTHROW(err1, Json::Error)
            << "Type of roi is not a list.";
    }

    Roi::list rois;
    for (const auto &v : value) {
        rois.emplace_back();
        auto &roi(rois.back());

        Json::get(roi.exploreUrl, v, "exploreUrl");
        Json::get(roi.roiUrl, v, "roiUrl");
        Json::get(roi.thumbnailUrl, v, "thumbnailUrl");
    }

    return rois;
}

Json::Value asJson(const View::map &namedViews
                   , BoundLayer::dict &boundLayers)
{
    Json::Value v(Json::objectValue);

    for (const auto &item : namedViews) {
        v[item.first] = asJson(item.second, boundLayers);
    }

    return v;
}

View::map namedViewsFromJson(const Json::Value &value)
{
    if (!value.isObject()) {
        LOGTHROW(err1, Json::Error)
            << "Type of named views is not an object.";
    }

    View::map namedViews;

    for (const auto &nvId : value.getMemberNames()) {
        namedViews[nvId] = viewFromJson(value[nvId]);
    }

    return namedViews;
}

void fromJson(ReferenceFrame &referenceFrame, const Json::Value &value)
{
    int version(0);
    Json::get(version, value, "version");

    switch (version) {
    case 1:
        v1::parse(referenceFrame, value);
        break;

    default:
        LOGTHROW(err1, storage::FormatError)
            << "Invalid reference frame JSON format: "
            "unsupported version " << version << ".";
    }
}

void fromJson(Srs::dict &srs, const Json::Value &value)
{
    parse(srs, value);
}

namespace {

const char* guessContentType(const boost::filesystem::path &path)
{
    auto e(path.extension().string());
    ba::to_upper(e);
    if (e == ".JPG") {
        return "image/jpeg";
    } else if (e == ".JPEG") {
        return "image/jpeg";
    } else if (e == ".PNG") {
        return "image/png";
    }
    return "application/octet-stream";
}
} // namespace

DataFile::DataFile(const boost::filesystem::path &path)
    : path(path), contentType(guessContentType(path))
{}

void ReferenceFrame::invalidate(const Division::Node::Id &nodeId)
{
    typedef ReferenceFrame::Division::Node Node;

    auto &nodes(division.nodes);
    for (auto inodes(nodes.begin()), enodes(nodes.end());
         inodes != enodes; )
    {
        if (isUnder(nodeId, inodes->first)) {
            // this node will be shielded by invalidated node, remove
            inodes = nodes.erase(inodes);
        } else {
            // skip
            ++inodes;
        }
    }

    // force invalid node
    nodes[nodeId] = Node(nodeId, PartitioningMode::none);
}

} } // namespace vadstena::registry
