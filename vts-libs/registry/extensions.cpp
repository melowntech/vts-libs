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

#include "../storage/error.hpp"

#include "detail/json.hpp"
#include "extensions/json.hpp"

namespace vtslibs { namespace registry { namespace extensions {

boost::any fromJson(const std::string &key, const Json::Value &value)
{
#ifndef VTSLIBS_BROWSER_ONLY
    if (key == Tms::key) {
        return tmsFromJson(value);
    } else if (key == Wmts::key) {
        return wmtsFromJson(value);
    }
#endif // VTSLIBS_BROWSER_ONLY

    return value;
}

Json::Value asJson(const boost::any &value)
{
#ifndef VTSLIBS_BROWSER_ONLY
    if (const auto *v = boost::any_cast<const Tms>(&value)) {
        return asJson(*v);
    }

    if (const auto *v = boost::any_cast<const Wmts>(&value)) {
        return asJson(*v);
    }
#endif // VTSLIBS_BROWSER_ONLY

    if (const auto *v = boost::any_cast<const Json::Value>(&value)) {
        return *v;
    }

    LOGTHROW(err2, storage::FormatError)
        << "Unknown extension type \"" << value.type().name() << "\"";
    throw;
}

} } } // namespace vtslibs::registry::extensions
