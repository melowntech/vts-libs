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
/**
 * \file storage/referenceframe.cpp
 * \author Vaclav Blazek <vaclav.blazek@citationtech.net>
 */

#include <algorithm>
#include <fstream>
#include <queue>
#include <numeric>

#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string/case_conv.hpp>
#include <boost/utility/in_place_factory.hpp>

#include "dbglog/dbglog.hpp"

#include "jsoncpp/json.hpp"
#include "jsoncpp/as.hpp"
#include "jsoncpp/io.hpp"

#include "../storage/error.hpp"
#include "referenceframe.hpp"
#include "json.hpp"
#include "detail/json.hpp"
#include "datafile.hpp"
#include "io.hpp"
#include "extensions.hpp"
#include "../registry.hpp"

namespace ba = boost::algorithm;

namespace vtslibs { namespace registry {

constexpr char Srs::typeName[];
constexpr char ReferenceFrame::typeName[];
constexpr char Credit::typeName[];
constexpr char BoundLayer::typeName[];
constexpr char DataFile::typeName[];
constexpr char Body::typeName[];

constexpr int BoundLayer::binaryOrder;
constexpr int BoundLayer::tileWidth;
constexpr int BoundLayer::tileHeight;
constexpr int BoundLayer::basicTileArea;

constexpr int BoundLayer::rasterMetatileBinaryOrder;
constexpr int BoundLayer::rasterMetatileWidth;
constexpr int BoundLayer::rasterMetatileHeight;
constexpr std::uint8_t BoundLayer::MetaFlags::watertight;
constexpr std::uint8_t BoundLayer::MetaFlags::available;
constexpr std::uint8_t BoundLayer::MetaFlags::unavailable;

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

void sanitize(const boost::filesystem::path &path
              , ReferenceFrame &rf);

namespace v1 {

void parse(ReferenceFrame::Division::Node::Partitioning &partitioning
           , const Json::Value &content)
{
    partitioning.mode = PartitioningMode::manual;

    math::Extents2 e;

    if (content.isMember("00")) {
        detail::parse(e, check(content["00"], Json::objectValue));
        partitioning.n00 = e;
    }

    if (content.isMember("01")) {
        detail::parse(e, check(content["01"], Json::objectValue));
        partitioning.n01 = e;
    }

    if (content.isMember("10")) {
        detail::parse(e, check(content["10"], Json::objectValue));
        partitioning.n10 = e;
    }

    if (content.isMember("11")) {
        detail::parse(e, check(content["11"], Json::objectValue));
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
    detail::parse(node.id, id);

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

    Json::getOpt(node.srs, content, "srs");

    if (content.isMember("extents")) {
        const auto &extents(content["extents"]);
        if (!extents.isObject()) {
            LOGTHROW(err1, Json::Error)
                << "Type of node(" << node.id
                << ")[extents] is not an object.";
        }
        detail::parse(node.extents, extents);
    }

    Json::getOpt(node.externalTexture, content, "externalTexture");
}

void parse(ReferenceFrame::Division &division, const Json::Value &content)
{
    const auto &extents(content["extents"]);
    if (!extents.isObject()) {
        LOGTHROW(err1, Json::Error)
            << "Type of division[extents] is not an object.";
    }
    detail::parse(division.extents, extents);
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

void parse(Extensions &ext, const Json::Value &content)
{
    for (const auto &name : content.getMemberNames()) {
        ext.insert(Extensions::value_type
                   (name, extensions::fromJson(name, content[name])));
    }
}

void parse(ReferenceFrame &rf, const Json::Value &content
           , const boost::filesystem::path &path)
{
    Json::get(rf.id, content, "id");
    Json::get(rf.description, content, "description");

    const auto &model(content["model"]);
    if (!model.isObject()) {
        LOGTHROW(err1, Json::Error)
            << "Type of referenceframe[model] is not an object.";
    }
    detail::parse(rf.model, model);
    Json::get(rf.body, content, "body");

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

    if (content.isMember("extensions")) {
        const auto &extensions(content["extensions"]);
        if (!extensions.isObject()) {
            LOGTHROW(err1, Json::Error)
                << "Type of referenceframe[extensions] is not an object.";
        }
        parse(rf.extensions, extensions);
    }

    sanitize(path, rf);
}

} // namespace v1


inline ReferenceFrame::Division::Node::Id
child(const ReferenceFrame::Division::Node::Id &id, bool x, bool y)
{
    return ReferenceFrame::Division::Node::Id
        (id.lod + 1, (id.x << 1) + x, (id.y << 1) + y);
}

/** Checks whether node is under root.
 */
inline bool isUnder(const ReferenceFrame::Division::Node::Id &root
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

inline ReferenceFrame::Division::Node::Id
parent(const ReferenceFrame::Division::Node::Id &nodeId
       , Lod diff = 1)
{
    if (diff > nodeId.lod) { return nodeId; }
    return ReferenceFrame::Division::Node::Id
        (nodeId.lod - diff, nodeId.x >> diff, nodeId.y >> diff);
}

inline ReferenceFrame::Division::Node::Structure::ChildFlags
childFlags(const ReferenceFrame::Division::Node::Id &childId)
{
    return 1 << ((childId.x & 1l) + 2 * (childId.y & 1l));
}

void sanitize(const boost::filesystem::path &path
              , ReferenceFrame &rf)
{
    typedef ReferenceFrame::Division::Node Node;
    typedef Node::Structure Structure;

    auto &div(rf.division);
    auto insertNode([&](const Node::Id &nodeId, PartitioningMode mode)
    {
        return &(div.nodes.insert
                 (Node::map::value_type
                  (nodeId, Node(nodeId, mode))).first->second);
    });

    // all found roots (i.e. nodes without parent)
    std::set<Node*> roots;

    // does this reference frame define proper root (i.e. 0-0-0)?
    const bool hasRoot(div.find({}, std::nothrow));

    // construct queue of nodes to process
    std::queue<Node*> nodes;
    for (auto &item : div.nodes) {
        nodes.push(&item.second);
    }

    // generated barren nodes
    std::vector<const Node*> barren;

    // process node queue
    while (!nodes.empty()) {
        // grab node
        auto *node(nodes.front());
        nodes.pop();
        auto &self(*node);
        auto &part(self.partitioning);
        const auto &id(self.id);

        // update lod range
        if (self.real()) {
            // productive node, remember in range of possible roots
            update(div.rootLodRange, id.lod);
        }

        if (id.lod) {
            // not a tree root -> find parent node
            const auto parentId(parent(id));
            auto *parentNode(div.find(parentId, std::nothrow));
            if (!parentNode && !hasRoot) {
                // no parent, this is (probably) one of the roots -> create new
                // barren (unproductive) node and add to the queue
                parentNode = insertNode
                    (parentId, PartitioningMode::barren);

                // remember barren node
                barren.push_back(parentNode);

                // remember in the queue (if not a tree root)
                if (parentId.lod) {
                    nodes.push(parentNode);
                }
            }

            if (parentNode) {
                // link structure: parent
                self.structure.parent = parentId;

                // and child
                parentNode->structure.children |= childFlags(id);
            }
        }

        if (part.mode == PartitioningMode::barren) {
            // skip barren (unproductive) node
            continue;
        }

        auto c00(child(id, 0, 0));
        auto c01(child(id, 0, 1));
        auto c10(child(id, 1, 0));
        auto c11(child(id, 1, 1));

        // find all node children
        auto *n00(div.find(c00, std::nothrow));
        auto *n01(div.find(c01, std::nothrow));
        auto *n10(div.find(c10, std::nothrow));
        auto *n11(div.find(c11, std::nothrow));

        if (part.mode == PartitioningMode::bisection) {
            // bisection of nothing at all -- there must be mothing inside
            // partitioning and no physical nodes under
            if (part.n00 || n00 || part.n01 || n01
                || part.n10 || n10 || part.n11 || n11)
            {
                LOGTHROW(err2, storage::FormatError)
                    << "Unable to parse reference frame file " << path
                    << ": reference frame <" << rf.id
                    << "> has invalid partitioning of node " << id
                    << " (either child node or partitioning "
                    "definition exists for bisection node).";
            }
            continue;
        }

        // manual partitioning -> check that at least child node exists, every
        // child node from partitioning exists and remember invalid children

        int left(4);
        auto check([&](const Node::Id &child
                       , const boost::optional<math::Extents2> &extents
                       , Node *childNode
                       , Structure::ChildFlags childFlag)
            mutable
        {
            if (bool(extents) != bool(childNode)) {
                LOGTHROW(err2, storage::FormatError)
                    << "Unable to parse reference frame file " << path
                    << ": reference frame <" << rf.id
                    << "> has invalid partitioning of node " << id
                    << " (mismatch in definition of child node "
                    << child << ").";
            }

            if (!extents) {
                // invalid child, generate
                insertNode(child, PartitioningMode::none);
                --left;
            } else {
                // create manual partition information in child
                childNode->constraints = boost::in_place(*extents, self.srs);
                // mark that this manually divided node has valid child down
                // there
                self.structure.children |= childFlag;
            }
        });

        check(c00, part.n00, n00, Structure::c00);
        check(c01, part.n01, n01, Structure::c01);
        check(c10, part.n10, n10, Structure::c10);
        check(c11, part.n11, n11, Structure::c11);

        if (!left) {
            LOGTHROW(err2, storage::FormatError)
                << "Unable to parse reference frame file " << path
                << ": reference frame <" << rf.id
                << "> has invalid partitioning of node " << id
                << " (no child node defined).";
        }
    }

    // postprocess barren nodes and generate invalid children
    for (const auto *node : barren) {
        const auto &id(node->id);
        const auto children(node->structure.children);

        if (!(children & Structure::c00)) {
            insertNode(child(id, false, false), PartitioningMode::none);
        }
        if (!(children & Structure::c01)) {
            insertNode(child(id, false, true), PartitioningMode::none);
        }
        if (!(children & Structure::c10)) {
            insertNode(child(id, true, false), PartitioningMode::none);
        }
        if (!(children & Structure::c11)) {
            insertNode(child(id, true, true), PartitioningMode::none);
        }
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
                v1::parse(rf, element, path);
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

        rfs.add(rf);
    }
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

    if (partitioning.n00) { detail::build(content["00"], *partitioning.n00); }
    if (partitioning.n01) { detail::build(content["01"], *partitioning.n01); }
    if (partitioning.n10) { detail::build(content["10"], *partitioning.n10); }
    if (partitioning.n11) { detail::build(content["11"], *partitioning.n11); }
}

void build(Json::Value &content, const ReferenceFrame::Division::Node &node)
{
    content = Json::objectValue;

    detail::build(content["id"], node.id);
    build(content["partitioning"], node.partitioning);

    if (node.partitioning.mode != PartitioningMode::none) {
        content["srs"] = node.srs;
        detail::build(content["extents"], node.extents);
    }

    // write only when true
    if (node.externalTexture) {
        content["externalTexture"] = node.externalTexture;
    }
}

void build(Json::Value &content, const ReferenceFrame::Division &division)
{
    content = Json::objectValue;

    detail::build(content["extents"], division.extents);

    auto &heightRange(content["heightRange"] = Json::arrayValue);
    heightRange.append(division.heightRange.min);
    heightRange.append(division.heightRange.max);

    auto &nodes(content["nodes"]);
    for (const auto &node : division.nodes) {
        // only real nodes are serialized
        if (node.second.real()) {
            build(nodes.append(Json::nullValue), node.second);
        }
    }
}

void build(Json::Value &content, const Extensions &ext)
{
    for (const auto &item : ext) {
        content[item.first] = extensions::asJson(item.second);
    }
}

void build(Json::Value &content, const ReferenceFrame &rf)
{
    content = Json::objectValue;
    content["version"] = DEFAULT_RF_VERSION;

    content["id"] = rf.id;
    content["description"] = rf.description;
    if (rf.body) { content["body"] = *rf.body; }

    detail::build(content["model"], rf.model);
    build(content["division"], rf.division);

    auto &parameters(content["parameters"] = Json::objectValue);
    parameters["metaBinaryOrder"] = rf.metaBinaryOrder;

    if (!rf.extensions.empty()) {
        build(content["extensions"], rf.extensions);
    }
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
    detail::parse(gg.extents, content["extents"]);

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

    Json::get(srs.alt, content, "alt");
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

        detail::build(jgg["extents"], gg.extents);

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

    if (srs.alt) { content["alt"] = *srs.alt; }
}

void build(Json::Value &content, const Srs::dict &srs
           , bool flatGeoidPath = false)
{
    content = Json::objectValue;

    for (const auto &s : srs) {
        build(content[s.first], s.second,  flatGeoidPath);
    }
}

/** NB: Do Not forget to update fromPython function in py.cpp!
 */
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
            credits.add(c);
        } catch (const Json::Error &e) {
            LOGTHROW(err1, storage::FormatError)
                << "Invalid credits collection format (" << e.what()
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
        build(content[c.id], c);
    }
}

void parse(Credits &credits, const Json::Value &value)
{
    if (value.isArray()) {
        for (const auto &element : value) {
            credits.set(element.asString(), boost::none);
        }
    } else if (value.isObject()) {
        for (const auto &id : Json::check(value, Json::objectValue)
                 .getMemberNames())
        {
            const auto &element(Json::check(value[id], Json::objectValue));
            if (element.empty()) {
                credits.set(id, boost::none);
            } else {
                Credit c;
                c.id = id;
                parse(c, element);
                credits.set(id, c);
            }
        }
    } else {
        LOGTHROW(err1, Json::Error)
            << "Type of credits is not a list nor an object.";
    }
}

/** NB: Do Not forget to update fromPython function in py.cpp!
 */
void parse(BoundLayer &bl, const Json::Value &content)
{
    Json::getOpt(bl.numericId, content, "id");

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

    // parse credits
    {
        const auto &cr(content["credits"]);
        if (cr.isString()) {
            // url
            bl.creditsUrl = cr.asString();
        } else {
            // credits
            parse(bl.credits, cr);
        }
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
            detail::parseIntSet(bla.codes, availability["codes"]
                                , "boundLayer[availability[codes]]");
            break;

        case BoundLayer::Availability::Type::negativeSize:
            Json::get(bla.size, availability, "size");
            break;
        }
    }

    // isTransparent is optional
    Json::getOpt(bl.isTransparent, content, "isTransparent");

    if (content.isMember("options")) { bl.options = content["options"]; }
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
            bls.add(bl);
        } catch (const Json::Error &e) {
            LOGTHROW(err1, storage::FormatError)
                << "Invalid bound layers collection format (" << e.what()
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

    if (bl.creditsUrl) {
        content["credits"] = *bl.creditsUrl;
    } else {
        build(content["credits"], bl.credits, inlineCredits);
    }

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

    if (!bl.options.empty()) {
        content["options"] = boost::any_cast<Json::Value>(bl.options);
    }
}

void build(Json::Value &content, const BoundLayer::dict &bls
           , bool inlineCredits = true)
{
    content = Json::objectValue;

    for (const auto &bl : bls) {
        if (bl.type == BoundLayer::Type::external) {
            // external bound layer: just URL
            content[bl.id] = bl.url;
        } else {
            // regular bound layer
            build(content[bl.id], bl, inlineCredits);
        }
    }
}

void parse(Body &b, const Json::Value &content)
{
    b.json = content;
    Json::get(b.parent, content, "parent");
    Json::get(b.defaultGeoidGrid, content, "defaultGeoidGrid");
}

void parse(Body::dict &bodies, const Json::Value &content)
{
    for (const auto &id : Json::check(content, Json::objectValue)
             .getMemberNames())
    {
        try {
            Body b;
            b.id = id;
            parse(b, Json::check(content[id], Json::objectValue));
            bodies.add(b);
        } catch (const Json::Error &e) {
            LOGTHROW(err1, storage::FormatError)
                << "Invalid bodies collection format (" << e.what()
                << ").";
        }
    }
}

void build(Json::Value &content, const Body &b)
{
    if (const auto json = boost::any_cast<Json::Value>(&b.json)) {
        content = Json::check(*json, Json::objectValue);
        return;
    }

    LOGTHROW(err1, storage::FormatError)
        << "Body <" << b.id << "> is not an Json::Value instance.";
}

void build(Json::Value &content, const Body::dict &bodies)
{
    content = Json::objectValue;

    for (const auto &b : bodies) {
        build(content[b.first], b.second);
    }
}

} // namesapce

ReferenceFrame::dict loadReferenceFrames(std::istream &in
                                         , const boost::filesystem::path &path)
{
    // load json
    auto content(Json::read<storage::FormatError>
                 (in, path, "reference frame"));

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
            << "Unable to load reference frame file " << path
            << ": <" << e.what() << ">.";
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
    Json::write(out, content);
}

void saveReferenceFrames(const boost::filesystem::path &path
                         , const ReferenceFrame::dict &rfs)
{
    LOG(info1) << "Saving reference frame file to " << path  << ".";
    std::ofstream f;
    try {
        f.exceptions(std::ios::badbit | std::ios::failbit);
        f.open(path.string(), std::ios_base::out);
    } catch (const std::exception &e) {
        LOGTHROW(err1, storage::IOError)
            << "Unable to save reference frame file " << path
            << ": <" << e.what() << ">.";
    }
    saveReferenceFrames(f, rfs);
    f.close();
}

Srs::dict loadSrs(std::istream &in, const boost::filesystem::path &path)
{
    // load json
    auto content(Json::read<storage::FormatError>
                 (in, path, "srs"));

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
            << "Unable to load srs file " << path
            << ": <" << e.what() << ">.";
    }
    auto srs(loadSrs(f, path));
    f.close();
    return srs;
}

void saveSrs(std::ostream &out, const Srs::dict &srs)
{
    Json::Value content;
    build(content, srs);
    out.precision(15);
    Json::write(out, content);
}

void saveSrs(const boost::filesystem::path &path
             , const Srs::dict &srs)
{
    LOG(info1) << "Saving srs file to " << path  << ".";
    std::ofstream f;
    try {
        f.exceptions(std::ios::badbit | std::ios::failbit);
        f.open(path.string(), std::ios_base::out);
    } catch (const std::exception &e) {
        LOGTHROW(err1, storage::IOError)
            << "Unable to save srs file " << path
            << ": <" << e.what() << ">.";
    }
    saveSrs(f, srs);
    f.close();
}

BoundLayer::dict loadBoundLayers(std::istream &in
                                 , const boost::filesystem::path &path)
{
    // load json
    auto content(Json::read<storage::FormatError>
                 (in, path, "bound layers"));

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
            << "Unable to load bound layer file " << path
            << ": <" << e.what() << ">.";
    }
    auto boundLayers(loadBoundLayers(f, path));
    f.close();
    return boundLayers;
}

void saveBoundLayers(std::ostream &out, const BoundLayer::dict &boundLayers)
{
    Json::Value content;
    build(content, boundLayers);
    out.precision(15);
    Json::write(out, content);
}

void saveBoundLayers(const boost::filesystem::path &path
                     , const BoundLayer::dict &boundLayers)
{
    LOG(info1) << "Saving bound layers file to " << path  << ".";
    std::ofstream f;
    try {
        f.exceptions(std::ios::badbit | std::ios::failbit);
        f.open(path.string(), std::ios_base::out);
    } catch (const std::exception &e) {
        LOGTHROW(err1, storage::IOError)
            << "Unable to save bound Layers file " << path
            << ": <" << e.what() << ">.";
    }
    saveBoundLayers(f, boundLayers);
    f.close();
}

void saveBoundLayer(std::ostream &out, const BoundLayer &boundLayer)
{
    Json::Value content;
    build(content, boundLayer);
    out.precision(15);
    Json::write(out, content);
}

BoundLayer loadBoundLayer(std::istream &in
                          , const boost::filesystem::path &path)
{
    // load json
    auto content(Json::read<storage::FormatError>
                 (in, path, "bound layer"));

    BoundLayer boundLayer;
    parse(boundLayer, content);
    return boundLayer;
}

Credit::dict loadCredits(std::istream &in
                         , const boost::filesystem::path &path)
{
    // load json
    auto content(Json::read<storage::FormatError>
                 (in, path, "credits"));

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
            << "Unable to load credits from file " << path
            << ": <" << e.what() << ">.";
    }
    auto credits(loadCredits(f, path));
    f.close();
    return credits;
}

void saveCredits(std::ostream &out, const Credit::dict &credits)
{
    Json::Value content;
    build(content, credits);
    out.precision(15);
    Json::write(out, content);
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
            << "Unable to save credits to file " << path
            << ": <" << e.what() << ">.";
    }
    saveCredits(f, credits);
    f.close();
}

void loadCredits(std::istream &in, Credits &credits
                 , const boost::filesystem::path &path)
{
    // load json
    auto content(Json::read<storage::FormatError>
                 (in, path, "credits"));

    parse(credits, content);
}

void saveCredits(std::ostream &out, const Credits &credits
                 , bool inlineCredits)
{
    Json::Value content;
    build(content, credits, inlineCredits);
    out.precision(15);
    Json::write(out, content);
}

Body::dict loadBodies(std::istream &in, const boost::filesystem::path &path)
{
    // load json
    auto content(Json::read<storage::FormatError>
                 (in, path, "bodies"));

    Body::dict bodies;
    parse(bodies, content);
    return bodies;
}

Body::dict loadBodies(const boost::filesystem::path &path)
{
    LOG(info1) << "Loading bodies file from " << path  << ".";
    std::ifstream f;
    f.exceptions(std::ios::badbit | std::ios::failbit);
    try {
        f.open(path.string(), std::ios_base::in);
    } catch (const std::exception &e) {
        LOGTHROW(err1, storage::IOError)
            << "Unable to load bodies from file " << path
            << ": <" << e.what() << ">.";
    }
    auto bodies(loadBodies(f, path));
    f.close();
    return bodies;
}

Body::dict loadBodies(const boost::filesystem::path &path, std::nothrow_t)
{
    LOG(info1) << "Loading bodies file from " << path  << ".";
    std::ifstream f;
    f.exceptions(std::ios::badbit | std::ios::failbit);
    try {
        f.open(path.string(), std::ios_base::in);
    } catch (const std::exception &e) {
        LOG(warn1)
            << "Unable to load bodies from file " << path
            << ": <" << e.what() << ">.";
        return {};
    }
    auto bodies(loadBodies(f, path));
    f.close();
    return bodies;
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
            << "No node <" << id << "> in the division tree.";
    }
    return *node;
}

std::set<std::string> ReferenceFrame::Division::srsList() const
{
    std::set<std::string> list;
    for (const auto &item : nodes) {
        if (item.second.real()) {
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

Json::Value asJson(const Credits &credits, bool inlineCredits)
{
    Json::Value content;
    build(content, credits, inlineCredits);
    return content;
}

void fromJson(Credits &credits, const Json::Value &value)
{
    parse(credits, value);
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

Json::Value asJson(const Body::dict &bodies)
{
    Json::Value content;
    build(content, bodies);
    return content;
}

Body::dict bodiesFromJson(const Json::Value &value)
{
    Body::dict bodies;
    parse(bodies, value);
    return bodies;
}

Position positionFromJson(const Json::Value &value)
{

    if (value.isString()) {
        auto str(value.asString());
        try {
            return boost::lexical_cast<Position>(str);
        } catch (const boost::bad_lexical_cast &) {
            LOGTHROW(err1, Json::Error)
                << "Unable to parse position from string \""
                << str << "\".";
        }
    }

    if (value.isArray()) {
        Position p;
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

    LOGTHROW(err1, Json::Error)
        << "Type of position is not a list nor a string.";
    throw;
}

Srs::dict listSrs(const ReferenceFrame &referenceFrame)
{
    Srs::dict srs;

    auto add([&](const std::string &id)
    {
        srs.set(id, system.srs(id));
    });

    add(referenceFrame.model.physicalSrs);
    add(referenceFrame.model.navigationSrs);
    add(referenceFrame.model.publicSrs);

    for (const auto &node : referenceFrame.division.nodes) {
        if (node.second.real()) {
            add(node.second.srs);
        }
    }

    return srs;
}

namespace {

void addBody(Body::dict &bodies, const std::string &bodyId)
{
    // do not allow infinite loops...
    if (bodies.has(bodyId)) { return; }

    if (const auto body = system.bodies(bodyId, std::nothrow)) {
        bodies.set(bodyId, *body);
        if (body->parent) { addBody(bodies, *body->parent); }
    }
}

void addParentBodyId(Body::IdList &bodies, const std::string &bodyId)
{
    if (const auto body = system.bodies(bodyId, std::nothrow)) {
        if (body->parent) {
            if (bodies.insert(*body->parent).second) {
                addParentBodyId(bodies, *body->parent);
            }
        }
    }
}

} // namespace

Body::dict listBodies(const ReferenceFrame &referenceFrame)
{
    if (!referenceFrame.body) { return {}; }
    Body::dict bodies;
    addBody(bodies, *referenceFrame.body);
    return bodies;
}

Body::IdList listParentBodies(const ReferenceFrame &referenceFrame)
{
    if (!referenceFrame.body) { return {}; }
    Body::IdList bodies;
    addParentBodyId(bodies, *referenceFrame.body);
    return bodies;
}

Credit::dict creditsAsDict(const StringIdSet &credits)
{
    Credit::dict c;
    for (const auto &id : credits) {
        c.add(system.credits(id, std::nothrow));
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
            c.add(system.credits(item.first, std::nothrow));
        }
    };
    return c;
}

Credit::dict creditsAsDict(const IdSet &credits)
{
    Credit::dict c;
    for (const auto &id : credits) {
        c.add(system.credits(id, std::nothrow));
    };
    return c;
}

BoundLayer::dict boundLayersAsDict(const StringIdSet &boundLayers)
{
    BoundLayer::dict b;
    for (const auto &id : boundLayers) {
        b.add(system.boundLayers(id, std::nothrow));
    };
    return b;
}

BoundLayer::dict boundLayersAsDict(const IdSet &boundLayers)
{
    BoundLayer::dict b;
    for (const auto &id : boundLayers) {
        b.add(system.boundLayers(id, std::nothrow));
    };
    return b;
}

const ReferenceFrame::Division::Node*
ReferenceFrame::Division::findSubtreeRoot(Node::Id nodeId, std::nothrow_t)
    const
{
    if (nodeId.lod < rootLodRange.min) {
        // OK, above possible roots -> either barren node or no node
        return find(nodeId, std::nothrow);
    }

    // if node is below max root lod move it inside
    if (nodeId.lod > rootLodRange.max) {
        nodeId = parent(nodeId, nodeId.lod - rootLodRange.max);
    }

    // try to find root
    for (;;) {
        // find node
        const Node *node(find(nodeId, std::nothrow));
        if (node || (nodeId.lod == rootLodRange.min)) {
            // either node is found or we are at the ceiling of root range
            return node;
        }

        // try parent
        nodeId = parent(nodeId);
    }
}

const ReferenceFrame::Division::Node&
ReferenceFrame::Division::findSubtreeRoot(const Node::Id &nodeId) const
{
    if (const auto *candidate = findSubtreeRoot(nodeId, std::nothrow)) {
        return *candidate;
    }

    LOGTHROW(err1, storage::KeyError)
        << "Node <" << nodeId << "> has no root in this reference frame.";
    throw;
}

Json::Value asJson(const View &view, BoundLayer::dict &boundLayers)
{
    Json::Value nv;

    if (view.description) {
        nv["description"] = *view.description;
    }
    auto &surfaces(nv["surfaces"] = Json::objectValue);

    auto addBoundLayers([&](Json::Value &out
                            , const View::BoundLayerParams::list &bls)
    {
        for (const auto &blp : bls) {
            if (blp.isComplex()) {
                auto &p(out.append(Json::objectValue));
                p["id"] = blp.id;
                if (blp.alpha) { p["alpha"] = *blp.alpha; }
                if (!blp.options.empty()) {
                    p["options"] = boost::any_cast<Json::Value>(blp.options);
                }
            } else {
                out.append(blp.id);
            }
            boundLayers.add(registry::system.boundLayers
                            (blp.id, std::nothrow));
        }
    });

    for (const auto &surfaceItem : view.surfaces) {
        addBoundLayers((surfaces[surfaceItem.first] = Json::arrayValue)
                       , surfaceItem.second);
    }

    auto &fls(nv["freeLayers"] = Json::objectValue);
    for (const auto &flItem : view.freeLayers) {
        auto &fl(fls[flItem.first] = Json::objectValue);

        const auto &params(flItem.second);
        if (params.style) { fl["style"] = *params.style; }
        if (!params.boundLayers.empty()) {
            addBoundLayers((fl["boundLayers"] = Json::arrayValue)
                           , params.boundLayers);
        }
        if (params.depthOffset) {
            auto &d(fl["depthOffset"] = Json::arrayValue);
            d.append((*params.depthOffset)[0]);
            d.append((*params.depthOffset)[1]);
            d.append((*params.depthOffset)[2]);
        }
        if (!params.options.empty()) {
            fl["options"] = boost::any_cast<Json::Value>(params.options);
        }
    }

    if (!view.bodies.empty()) {
        auto &bodies(nv["bodies"] = Json::arrayValue);
        for (const auto &body : view.bodies) {
            bodies.append(body);
        }
    }

    if (!view.options.empty()) {
        try {
            nv["options"] = boost::any_cast<Json::Value>(view.options);
        } catch (const boost::bad_any_cast&) {
            // ignore
        }
    }

    return nv;
}

void fromJson(View &view, const Json::Value &value)
{
    if (!value.isObject()) {
        LOGTHROW(err1, Json::Error)
            << "Type of view is not an object.";
    }

    Json::get(view.description, value, "description");

    const auto &surfaces(value["surfaces"]);
    if (!surfaces.isObject()) {
        LOGTHROW(err1, Json::Error)
            << "Type of namedView[surfaces] is not an object.";
    }

    auto addBoundLayers([&](View::BoundLayerParams::list &bls
                            , const Json::Value &in)
    {
        for (const auto &blp : in) {
            if (blp.isObject()) {
                bls.emplace_back();
                auto &item(bls.back());
                Json::get(item.id, blp, "id");
                Json::get(item.alpha, blp, "alpha");
                if (blp.isMember("options")) { item.options = blp["options"]; }
            } else if (blp.isString()) {
                bls.push_back(blp.asString());
            } else {
                LOGTHROW(err1, Json::Error)
                    << "Type of boundLayer must be either string or object.";
            }
        }
    });

    for (const auto &sId : surfaces.getMemberNames()) {
        const auto &surface(surfaces[sId]);
        if (!surface.isArray()) {
            LOGTHROW(err1, Json::Error)
                << "Type of namedView[surfaces][" << sId
                << "] is not a list.";
        }

        addBoundLayers(view.surfaces[sId], surface);
    }

    const auto &freeLayers(value["freeLayers"]);
    if (!freeLayers.isNull()) {
        if (!freeLayers.isObject()) {
            LOGTHROW(err1, Json::Error)
                << "Type of view[freeLayers] member is not an object.";
        }

        for (const auto &fId : freeLayers.getMemberNames()) {
            auto &jfl(freeLayers[fId]);
            auto &fl(view.freeLayers[fId]);
            if (jfl.isMember("boundLayers")) {
                // add bound layers
                addBoundLayers(fl.boundLayers, jfl["boundLayers"]);
            }
            Json::get(fl.style, jfl, "style");

            if (jfl.isMember("depthOffset")) {
                const auto &d(jfl["depthOffset"]);
                if (!d.isArray()) {
                    LOGTHROW(err1, Json::Error)
                        << "Type of view[freeLayers].depthOffset is "
                        "not an array.";
                }

                fl.depthOffset = boost::in_place();
                (*fl.depthOffset)[0] = d[0].asDouble();
                (*fl.depthOffset)[1] = d[1].asDouble();
                (*fl.depthOffset)[2] = d[2].asDouble();
            }

            if (jfl.isMember("options")) { fl.options = jfl["options"]; }
        }
    }

    const auto &bodies(value["bodies"]);
    if (!bodies.isNull()) {
        if (!bodies.isArray()) {
            LOGTHROW(err1, Json::Error)
                << "Type of view[bodies] member is not an array.";
        }
        for (const auto &body : bodies) {
            view.bodies.insert(body.asString());
        }
    }

    {
        auto options(value["options"]);
        if (!options.isNull()) { view.options = options; }
    }
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

void fromJson(View::map &views, const Json::Value &value)
{
    if (!value.isObject()) {
        LOGTHROW(err1, Json::Error)
            << "Type of named views is not an object.";
    }

    for (const auto &nvId : value.getMemberNames()) {
        views[nvId] = viewFromJson(value[nvId]);
    }
}

void fromJson(ReferenceFrame &referenceFrame, const Json::Value &value
              , const boost::filesystem::path &path)
{
    int version(0);
    Json::get(version, value, "version");

    switch (version) {
    case 1:
        v1::parse(referenceFrame, value, path);
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

Json::Value asJson(const RegistryBase &rb)
{
    Json::Value value(Json::objectValue);
    if (!rb.credits.empty()) {
        build(value["credits"], rb.credits);
    }
    if (!rb.boundLayers.empty()) {
        build(value["boundLayers"], rb.boundLayers);
    }
    return value;
}

void fromJson(RegistryBase &rb, const Json::Value &value)
{
    // parse value
    if (value.isMember("credits")) {
        parse(rb.credits, value["credits"]);
    }
    if (value.isMember("boundLayers")) {
        parse(rb.boundLayers, value["boundLayers"]);
    }
}

void load(RegistryBase &rb, std::istream &in
          , const boost::filesystem::path &path)
{
    // load json
    auto content(Json::read<storage::FormatError>
                 (in, path, "registry base"));

    fromJson(rb, content);
}

void load(RegistryBase &rb, const boost::filesystem::path &path)
{
    LOG(info1) << "Loading registry base from file from " << path  << ".";
    std::ifstream f;
    f.exceptions(std::ios::badbit | std::ios::failbit);
    try {
        f.open(path.string(), std::ios_base::in);
    } catch (const std::exception &e) {
        LOGTHROW(err1, storage::IOError)
            << "Unable to load registry base file " << path
            << ": <" << e.what() << ">.";
    }

    load(rb, f, path);
    f.close();
}

void save(std::ostream &out, const RegistryBase &rb)
{
    out.precision(15);
    Json::write(out, asJson(rb));
}

void save(const boost::filesystem::path &path, const RegistryBase &rb)
{
    LOG(info1) << "Saving registry base file to " << path  << ".";
    std::ofstream f;
    try {
        f.exceptions(std::ios::badbit | std::ios::failbit);
        f.open(path.string(), std::ios_base::out);
    } catch (const std::exception &e) {
        LOGTHROW(err1, storage::IOError)
            << "Unable to save registry base file " << path
            << ": <" << e.what() << ">.";
    }
    save(f, rb);
    f.close();
}

namespace {

struct Absolutize {
    Absolutize(const utility::Uri &base) : base(base) {}

    const utility::Uri &base;

    void absolutize(std::string &url) const {
        url = base.resolve(url).str();
    }

    void absolutize(boost::optional<std::string> &url) const {
        if (url) { absolutize(*url); }
    }

    void absolutize(registry::BoundLayer &bl) const {
        absolutize(bl.url);
        absolutize(bl.maskUrl);
        absolutize(bl.metaUrl);
        absolutize(bl.creditsUrl);
    }
};

} // namespace

BoundLayer absolutize(const BoundLayer &boundLayer
                      , const utility::Uri &baseUrl)
{
    auto bl(boundLayer);
    Absolutize(baseUrl).absolutize(bl);
    return bl;
}

#define REGISTRY_COMPARE_DIFFERS(MEMBER)        \
    if (l.MEMBER != r.MEMBER) { return false; }

bool operator==(const Credit &l, const Credit &r)
{
    REGISTRY_COMPARE_DIFFERS(id)
    REGISTRY_COMPARE_DIFFERS(numericId)
    REGISTRY_COMPARE_DIFFERS(notice)
    REGISTRY_COMPARE_DIFFERS(url)
    REGISTRY_COMPARE_DIFFERS(copyrighted)
    return true;
}

bool operator==(const BoundLayer::Availability &l
                , const BoundLayer::Availability &r)
{
    REGISTRY_COMPARE_DIFFERS(type)
    REGISTRY_COMPARE_DIFFERS(mime)
    REGISTRY_COMPARE_DIFFERS(codes)
    REGISTRY_COMPARE_DIFFERS(size)
    return true;
}

bool operator==(const BoundLayer &l, const BoundLayer &r)
{
    REGISTRY_COMPARE_DIFFERS(id)
    REGISTRY_COMPARE_DIFFERS(numericId)
    REGISTRY_COMPARE_DIFFERS(type)
    REGISTRY_COMPARE_DIFFERS(url)
    REGISTRY_COMPARE_DIFFERS(maskUrl)
    REGISTRY_COMPARE_DIFFERS(metaUrl)
    REGISTRY_COMPARE_DIFFERS(creditsUrl)
    REGISTRY_COMPARE_DIFFERS(lodRange)
    REGISTRY_COMPARE_DIFFERS(tileRange)
    REGISTRY_COMPARE_DIFFERS(credits)
    REGISTRY_COMPARE_DIFFERS(availability)
    REGISTRY_COMPARE_DIFFERS(isTransparent)

    // options:
    {
        // try to get options
        const auto lo(boost::any_cast<Json::Value>(&l.options));
        const auto ro(boost::any_cast<Json::Value>(&r.options));

        // different presence
        if (bool(lo) != bool(ro)) { return false; }
        // check for difference if both present
        if (lo && (*lo != *ro)) { return false; }
    }

    return true;
}

#undef REGISTRY_COMPARE_DIFFERS

} } // namespace vtslibs::registry
