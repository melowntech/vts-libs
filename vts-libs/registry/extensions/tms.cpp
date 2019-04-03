/**
 * Copyright (c) 2019 Melown Technologies SE
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
 * \file registry/extensions/tms.cpp
 * \author Vaclav Blazek <vaclav.blazek@melown.com>
 */

#include <boost/lexical_cast.hpp>

#include "dbglog/dbglog.hpp"

#include "jsoncpp/json.hpp"
#include "jsoncpp/as.hpp"
#include "jsoncpp/io.hpp"

#include "../../storage/error.hpp"
#include "../detail/json.hpp"

#include "json.hpp"
#include "tms.hpp"

namespace vtslibs { namespace registry { namespace extensions {

constexpr char Tms::key[];

namespace {

void parse(Tms &tms, const Json::Value &value)
{
    if (value.isMember("rootId")) {
        const auto &rootId(value["rootId"]);
        if (!rootId.isObject()) {
            LOGTHROW(err1, Json::Error)
                << "Type of referenceframe[extentsions[tms].rootId] is not "
                "an object.";
        }
        detail::parse(tms.rootId, rootId);
    }
    Json::getOpt(tms.flipY, value, "flipY");
    Json::get(tms.profile, value, "profile");
    Json::get(tms.physicalSrs, value, "physicalSrs");
    Json::get(tms.projection, value, "projection");
}

} // namespace

Tms tmsFromJson(const Json::Value &value)
{
    Tms tms;
    parse(tms, value);
    return tms;
}

Json::Value asJson(const Tms &tms)
{
    Json::Value value(Json::objectValue);
    if (tms.rootId != registry::ReferenceFrame::Division::Node::Id()) {
        detail::build(value["rootId"], tms.rootId);
    }
    if (!tms.flipY) { value["flipY"] = tms.flipY; }
    value["profile"] = boost::lexical_cast<std::string>(tms.profile);
    if (tms.physicalSrs) { value["physicalSrs"] = *tms.physicalSrs; }
    value["projection"] = tms.projection;
    return value;
}

void load(Tms &tms, std::istream &is)
{
    parse(tms, Json::read(is));
}

void save(const Tms &tms, std::ostream &os)
{
    Json::write(os, asJson(tms));
}

} } } // namespace vtslibs::registry::extensions
