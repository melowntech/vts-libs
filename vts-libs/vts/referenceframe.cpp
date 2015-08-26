/**
 * \file vts/referenceframe.cpp
 * \author Vaclav Blazek <vaclav.blazek@citationtech.net>
 */

#include <fstream>

#include "dbglog/dbglog.hpp"

#include "jsoncpp/json.hpp"

#include "../storage/error.hpp"
#include "./referenceframe.hpp"

namespace vadstena { namespace vts {

namespace {

void parse(ReferenceFrame::dict &rfs, const Json::Value &content)
{
    (void) rfs;
    (void) content;
    // TODO: implement me
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
            << "Unable to parse reference frame config: "
            << reader.getFormattedErrorMessages() << ".";
    }

    ReferenceFrame::dict rfs;
    parse(rfs, content);
    return rfs;
}

ReferenceFrame::dict loadReferenceFrames(const boost::filesystem::path &path)
{
    LOG(info1) << "Loading reference frame config from " << path  << ".";
    std::ifstream f;
    f.exceptions(std::ios::badbit | std::ios::failbit);
    try {
        f.open(path.string(), std::ios_base::in);
    } catch (const std::exception &e) {
        LOGTHROW(err1, storage::IOError)
            << "Unable to load reference frame config file " << path << ".";
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
    LOG(info1) << "Saving reference frame config file to " << path  << ".";
    std::ofstream f;
    try {
        f.exceptions(std::ios::badbit | std::ios::failbit);
        f.open(path.string(), std::ios_base::out);
    } catch (const std::exception &e) {
        LOGTHROW(err1, storage::IOError)
            << "Unable to save reference frame config file " << path << ".";
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
            << "Unable to parse srs config: "
            << reader.getFormattedErrorMessages() << ".";
    }

    Srs::dict srs;
    parse(srs, content);
    return srs;
}

Srs::dict loadSrs(const boost::filesystem::path &path)
{
    LOG(info1) << "Loading srs config from " << path  << ".";
    std::ifstream f;
    f.exceptions(std::ios::badbit | std::ios::failbit);
    try {
        f.open(path.string(), std::ios_base::in);
    } catch (const std::exception &e) {
        LOGTHROW(err1, storage::IOError)
            << "Unable to load srs config file " << path << ".";
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
    LOG(info1) << "Saving srs config file to " << path  << ".";
    std::ofstream f;
    try {
        f.exceptions(std::ios::badbit | std::ios::failbit);
        f.open(path.string(), std::ios_base::out);
    } catch (const std::exception &e) {
        LOGTHROW(err1, storage::IOError)
            << "Unable to save srs config file " << path << ".";
    }
    saveSrs(f, srs);
    f.close();
}

} } // namespace vadstena::vts
