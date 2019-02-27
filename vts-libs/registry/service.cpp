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

#include <algorithm>
#include <fstream>
#include <queue>
#include <numeric>

#include "dbglog/dbglog.hpp"

#include "jsoncpp/json.hpp"
#include "jsoncpp/as.hpp"
#include "jsoncpp/io.hpp"

#include "../storage/error.hpp"
#include "service.hpp"
#include "json.hpp"


namespace vtslibs { namespace registry {

constexpr char Service::typeName[];

namespace {

void build(Json::Value &content, const Service &service)
{
    if (service.params.empty()) {
        content = Json::objectValue;
    } else {
        content = *boost::any_cast<const Json::Value>(&service.params);
    }
    content["url"] = service.url;
}

void build(Json::Value &content, const Service::dict &services)
{
    content = Json::objectValue;

    for (const auto &item : services) {
        const auto &id(item.first);
        const auto &service(item.second);

        // regular free layer
        build(content[id], service);
    }
}

void parse(Service &service, const Json::Value &content)
{
    if (content.isString()) {
        service.url = content.asString();
        service.params = {};
        return;
    }

    if (!content.isObject()) {
        LOGTHROW(err1, storage::FormatError)
            << "Invalid service definition: must be either an object "
            "or a sttring.";
    }

    Json::get(service.url, content, "url");
    service.params = content;
}

void parse(Service::dict &services, const Json::Value &content)
{
    for (const auto &id : Json::check(content, Json::objectValue)
             .getMemberNames())
    {
        try {
            Service service;
            service.id = id;
            parse(service, content[id]);
            services.set(id, service);
        } catch (const Json::Error &e) {
            LOGTHROW(err1, storage::FormatError)
                << "Invalid service file format (" << e.what()
                << ").";
        }
    }
}

} // namesapce

Json::Value asJson(const Service::dict &service)
{
    Json::Value content;
    build(content, service);
    return content;
}

Json::Value asJson(const Service &service)
{
    Json::Value content;
    build(content, service);
    return content;
}

void fromJson(Service::dict &services, const Json::Value &value)
{
    parse(services, value);
}

} } // namespace vtslibs::registry
