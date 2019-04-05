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
 * \file registry/extensions/wmts.cpp
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
#include "wmts.hpp"

namespace vtslibs { namespace registry { namespace extensions {

constexpr char Wmts::key[];

namespace {

void parse(Wmts &wmts, const Json::Value &value)
{
    Json::get(wmts.extentsSrs, value, "extentsSrs");
    Json::get(wmts.content, value, "content");
    Json::get(wmts.projection, value, "projection");
    Json::get(wmts.wellKnownScaleSet, value, "wellKnownScaleSet");
}

} // namespace

Wmts wmtsFromJson(const Json::Value &value)
{
    Wmts wmts;
    parse(wmts, value);
    return wmts;
}

Json::Value asJson(const Wmts &wmts)
{
    Json::Value value(Json::objectValue);
    if (wmts.extentsSrs) { value["extentsSrs"] = *wmts.extentsSrs; }
    if (wmts.content) { value["content"] = *wmts.content; }
    value["projection"] = wmts.projection;
    if (wmts.wellKnownScaleSet) {
        value["wellKnownScaleSet"] = *wmts.wellKnownScaleSet;
    }
    return value;
}

void load(Wmts &wmts, std::istream &is)
{
    parse(wmts, Json::read(is));
}

void save(const Wmts &wmts, std::ostream &os)
{
    Json::write(os, asJson(wmts));
}

} } } // namespace vtslibs::registry::extensions
