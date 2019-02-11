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

#ifndef vtslibs_registry_urlexpander_hpp_included_
#define vtslibs_registry_urlexpander_hpp_included_

#include <functional>

#include "utility/uri.hpp"

#include "referenceframe.hpp"
#include "freelayer.hpp"

namespace vtslibs { namespace registry {

/** Expands URLs in various entities.
 */
class UrlExpander {
public:
    typedef std::function<std::string(const std::string &url)> Fetcher;

    UrlExpander(const utility::Uri &base, const Fetcher &fetcher);

    BoundLayer expand(const BoundLayer &bl) const;

    BoundLayer::dict expand(const BoundLayer::dict &boundLayers) const;

    FreeLayer expand(const FreeLayer &fl) const;

    FreeLayer::dict expand(const FreeLayer::dict &freeLayers) const;

private:
    utility::Uri absolute(const std::string &url) const;

    std::string fetch(utility::Uri url) const;

    utility::Uri base_;
    Fetcher fetcher_;
};

} } // namespace vtslibs::registry

#endif // vtslibs_registry_urlexpander_hpp_included_
