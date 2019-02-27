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

#include "http/ondemandclient.hpp"
#include "http/error.hpp"

#include "../../storage/error.hpp"

#include "urlfetcher.hpp"

namespace vtslibs { namespace vts { namespace tools {

UrlFetcher::UrlFetcher(long timeout, int threads)
    : timeout_(timeout), client_(new http::OnDemandClient(threads))
{}

UrlFetcher::~UrlFetcher() {}

std::string UrlFetcher::fetch(const std::string &url) const
{
    auto q(client_->fetcher().perform
           (utility::ResourceFetcher::Query(url)
            .timeout(timeout_)));

    if (q.ec()) {
        if (q.check(make_error_code(utility::HttpCode::NotFound))) {
            LOGTHROW(err2, storage::NoSuchFile)
                << "File at URL <" << url << "> doesn't exist.";
            return {};
        }
        LOGTHROW(err1, storage::IOError)
            << "Failed to download tile data from <"
            << url << ">: Unexpected HTTP status code: <"
            << q.ec() << ">.";
    }

    try {
        return std::move(q.moveOut().data);
    } catch (const http::Error &e) {
        LOGTHROW(err1, storage::IOError)
            << "Failed to download tile data from <"
            << url << ">: Unexpected error code <"
            << e.what() << ">.";
    }
    throw;
}

} } } // namespace vtslibs::vts::tools
