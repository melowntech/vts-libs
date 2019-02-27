/**
 * Copyright (c) 2018 Melown Technologies SE
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

#include "urlexpander.hpp"

namespace vtslibs { namespace registry {

UrlExpander::UrlExpander(const utility::Uri &base, const Fetcher &fetcher)
    : base_(base), fetcher_(fetcher)
{
}

utility::Uri UrlExpander::absolute(const std::string &url) const {
    return base_.resolve(url);
}

std::string UrlExpander::fetch(utility::Uri url) const
{
    if (url.scheme().empty()) { url.scheme("http"); }
    return fetcher_(url.str());
}

BoundLayer UrlExpander::expand(const BoundLayer &bl) const
{
    if (bl.type != BoundLayer::Type::external) { return bl; }

    const auto url(absolute(bl.url));
    std::istringstream is(fetch(url));
    auto out(absolutize(loadBoundLayer(is, url.str()), url));
    out.id = bl.id;
    return out;
}

BoundLayer::dict UrlExpander::expand(const BoundLayer::dict &boundLayers) const
{
    BoundLayer::dict out;
    for (auto bl : boundLayers) {
        out.add(expand(bl));
    }
    return out;
}

FreeLayer UrlExpander::expand(const FreeLayer &fl) const
{
    if (auto *flUrl = boost::get<std::string>(&fl.definition)) {
        const auto url(absolute(*flUrl));
        std::istringstream is(fetch(url));
        auto out(absolutize(loadFreeLayer(is, url.str()), url));
        out.id = fl.id;
        return out;
    }
    return fl;
}

FreeLayer::dict UrlExpander::expand(const FreeLayer::dict &freeLayers) const
{
    FreeLayer::dict out;
    freeLayers.for_each([&](const FreeLayer &fl)
    {
        out.add(expand(fl));
    });
    return out;
}

} } // namespace vtslibs::registry
