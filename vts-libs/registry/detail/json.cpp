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
 * \file registry/detail/json.cpp
 * \author Vaclav Blazek <vaclav.blazek@citationtech.net>
 */

#include "jsoncpp/as.hpp"

#include "json.hpp"

namespace vtslibs { namespace registry { namespace detail {

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

} } } // namespace vtslibs::registry::detail
