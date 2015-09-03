/**
 * \file storage/referenceframe.cpp
 * \author Vaclav Blazek <vaclav.blazek@citationtech.net>
 */

#include <fstream>

#include <boost/lexical_cast.hpp>

#include "dbglog/dbglog.hpp"

#include "jsoncpp/json.hpp"
#include "jsoncpp/as.hpp"

#include "../storage/error.hpp"
#include "./referenceframe.hpp"

namespace vadstena { namespace storage {

namespace {

namespace v1 {

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
}

void parse(ReferenceFrame::Division &division, const Json::Value &content)
{
    const auto &extents(content["extents"]);
    if (!extents.isObject()) {
        LOGTHROW(err1, Json::Error)
            << "Type of division[extents] is not an object.";
    }
    parse(division.extents, extents);

    Json::get(division.rootLod, content, "rootLod");
    Json::get(division.arity, content, "arity");

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
}

} // namespace v1

void parse(ReferenceFrame::dict &rfs, const Json::Value &content)
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
        rfs.add(rf);
    }
}

void build(Json::Value &content, const ReferenceFrame::dict &rfs)
{
    (void) rfs;
    (void) content;
    // TODO: implement me
}

void parse(Srs::dict &srs, const Json::Value &content)
{
    (void) srs;
    (void) content;
    // TODO: implement me
}

void build(Json::Value &content, const Srs::dict &srs)
{
    (void) srs;
    (void) content;
    // TODO: implement me
}

} // namesapce

ReferenceFrame::dict loadReferenceFrames(std::istream &in)
{
    // load json
    Json::Value content;
    Json::Reader reader;
    if (!reader.parse(in, content)) {
        LOGTHROW(err2, storage::FormatError)
            << "Unable to parse reference frame file: "
            << reader.getFormattedErrorMessages() << ".";
    }

    ReferenceFrame::dict rfs;
    parse(rfs, content);
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
    auto rfs(loadReferenceFrames(f));
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

} } // namespace vadstena::storage
