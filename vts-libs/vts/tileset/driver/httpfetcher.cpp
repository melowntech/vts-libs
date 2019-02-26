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
#include <iterator>
#include <future>

#include <boost/format.hpp>
#include <boost/iostreams/device/array.hpp>
#include <boost/algorithm/string/predicate.hpp>

#include <curl/curl.h>

#include "utility/uri.hpp"

#include "http/ondemandclient.hpp"
#include "http/error.hpp"

#include "../../../storage/error.hpp"
#include "../../../storage/sstreams.hpp"
#include "../../tileop.hpp"
#include "httpfetcher.hpp"
#include "runcallback.hpp"

namespace ba = boost::algorithm;

namespace vtslibs { namespace vts { namespace driver {

namespace {

const std::string ConfigName("tileset.conf");
const std::string ExtraConfigName("extra.conf");
const std::string TileIndexName("tileset.index");
const std::string RegistryName("tileset.registry");

const std::string filePath(File type)
{
    switch (type) {
    case File::config: return ConfigName;
    case File::extraConfig: return ExtraConfigName;
    case File::tileIndex: return TileIndexName;
    case File::registry: return RegistryName;
    default: break;
    }
    throw "unknown file type";
}

const std::string remotePath(const TileId &tileId, TileFile type
                             , unsigned int revision)
{
    const char *ext([&]() -> const char*
    {
        switch (type) {
        case TileFile::meta: return ".meta";
        case TileFile::mesh: return ".rmesh";
        case TileFile::atlas: return ".ratlas";
        case TileFile::navtile: return ".rnavtile";
        default: break;
        }
        throw "Unexpected TileFile value. Go fix your program.";
    }());

    return str(boost::format("%d-%d-%d%s?%d")
               % tileId.lod % tileId.x % tileId.y % ext % revision);
}

std::string joinUrl(std::string url, const std::string &filename)
{
    if (!url.empty() && (url.back() != '/')) {
        url.push_back('/');
    }
    url.append(filename);
    return url;
}

http::OnDemandClient sharedClient;

utility::ResourceFetcher& getFetcher(const OpenOptions &options) {
    if (const auto &fetcher = options.resourceFetcher()) {
        return *fetcher;
    }
    return sharedClient.fetcher();
}

IStream::pointer fetchAsStream(const std::string &rootUrl
                               , const std::string &filename
                               , const char *contentType
                               , const OpenOptions &options
                               , bool noSuchFile)
{
    const auto &fetcher(getFetcher(options));

    const std::string url(joinUrl(rootUrl, filename));

    auto tryFetch([&](unsigned long delay) -> IStream::pointer
    {
        auto q(fetcher.perform
               (utility::ResourceFetcher::Query(url)
                .timeout(options.ioWait()).delay(delay)));

        if (q.ec()) {
            if (q.check(make_error_code(utility::HttpCode::NotFound))) {
                if (noSuchFile) {
                    LOGTHROW(err2, storage::NoSuchFile)
                        << "File at URL <" << url << "> doesn't exist.";
                }
                return {};
            }

            LOGTHROW(err1, storage::IOError)
                << "Failed to download tile data from <"
                << url << ">: Unexpected HTTP status code: <"
                << q.ec() << ">.";
        }

        try {
            auto body(q.moveOut());
            return storage::memIStream(contentType, std::move(body.data)
                                       , body.lastModified, url);
        } catch (const http::Error &e) {
            LOGTHROW(err1, storage::IOError)
                << "Failed to download tile data from <"
                << url << ">: Unexpected error code <"
                << e.what() << ">.";
        }
        return {};
    });

    unsigned long delay(0);
    for (auto tries(options.ioRetries()); tries; (tries > 0) ? --tries : 0) {
        try {
            return tryFetch(delay);
        } catch (const storage::IOError &e) {
            LOG(warn2) << "Failed to fetch file from <" << url
                       << ">; retrying in " << options.ioRetryDelay()
                       << " ms.";
            delay = options.ioRetryDelay();
        }
    }
    return tryFetch(delay);
}

class AsyncFetcher
    : public std::enable_shared_from_this<AsyncFetcher>
{
public:
    AsyncFetcher(const std::string &url, const char *contentType
                 , const OpenOptions &options, const InputCallback &cb
                 , const IStream::pointer *notFound)
        : fetcher_(getFetcher(options))
        , url_(url), contentType_(contentType)
        , cb_(cb), notFound_(notFound)
        , ioWait_(options.ioWait())
        , ioRetryDelay_(options.ioRetryDelay())
        , triesLeft_(options.ioRetries())
    {}

    void run(unsigned long delay = 0);

private:
    typedef utility::ResourceFetcher::Query Query;
    typedef utility::ResourceFetcher::MultiQuery MultiQuery;

    void queryDone(MultiQuery &&query);

    utility::ResourceFetcher &fetcher_;
    const std::string url_;
    const char *contentType_;
    InputCallback cb_;
    const IStream::pointer *notFound_;
    const long ioWait_;
    const unsigned long ioRetryDelay_;
    int triesLeft_;
};

void AsyncFetcher::run(unsigned long delay)
{
    fetcher_.perform(Query(url_).timeout(ioWait_).delay(delay)
                     , std::bind(&AsyncFetcher::queryDone
                                 , shared_from_this()
                                 , std::placeholders::_1));
}

void AsyncFetcher::queryDone(MultiQuery &&mq)
{
    auto &q(mq.front());

    try {
        if (q.ec()) {
            if (q.check(make_error_code(utility::HttpCode::NotFound))) {
                if (notFound_) {
                    return runCallback([&]() { return *notFound_; }, cb_);
                }
                LOGTHROW(err2, storage::NoSuchFile)
                    << "File at URL <" << url_ << "> doesn't exist.";
            }

            LOGTHROW(err1, storage::IOError)
                << "Failed to download tile data from <"
                << url_ << ">: Unexpected HTTP status code: <"
                << q.ec() << ">.";
        }

        try {
            auto body(q.moveOut());
            return runCallback([&]()
            {
                return storage::memIStream(contentType_, std::move(body.data)
                                           , body.lastModified, url_);
            }, cb_);
        } catch (const http::Error &e) {
            LOGTHROW(err1, storage::IOError)
                << "Failed to download tile data from <"
                << url_ << ">: Unexpected error code <"
                << e.what() << ">.";
        }

    } catch (...) {
        // we do not touch negative number of tries, otherwise decrement
        if (triesLeft_ > 0) { --triesLeft_; }
        if (!triesLeft_) {
            // no try left -> forward error
            return runCallback([&]()
            {
                return std::current_exception();
            }, cb_);
        }
    }

    // failed, restart
    LOG(warn2) << "Failed to fetch file from <" << url_
               << ">; retrying in " << ioRetryDelay_ << " ms.";
    run(ioRetryDelay_);
}

std::string fixUrl(const std::string &input, const OpenOptions &options)
{
    utility::Uri uri(input);
    if (!uri.absolute()) {
        LOGTHROW(err2, storage::IOError)
            << "Uri <" << input << "> is not absolute uri.";
    }

    const auto &cnames(options.cnames());

    if (!uri.scheme().empty() && cnames.empty()) {
        // nothing to be changed, return as-is
        return input;
    }

    // something has to be changed

    // no scheme, both http and https should work, force http
    if (uri.scheme().empty()) {
        uri.scheme("http");
    }

    if (!cnames.empty()) {
        // apply cnames, case-insensitive way
        for (const auto &item : cnames) {
            if (ba::iequals(uri.host(), item.first)) {
                uri.host(item.second);
                break;
            }
        }
    }

    return str(uri);
}

} // namespace

HttpFetcher::HttpFetcher(const std::string &rootUrl
                         , const OpenOptions &options)
    : rootUrl_(fixUrl(rootUrl, options))
    , options_(options)
{
    LOG(info1) << "Using URI: <" << rootUrl_ << ">.";
}

IStream::pointer HttpFetcher::input(File type, bool noSuchFile)
    const
{
    return fetchAsStream(rootUrl_, filePath(type), contentType(type), options_
                         , noSuchFile);
}

IStream::pointer HttpFetcher::input(const TileId &tileId, TileFile type
                                    , unsigned int revision, bool noSuchFile)
    const
{
    return fetchAsStream(rootUrl_, remotePath(tileId, type, revision)
                         , contentType(type), options_, noSuchFile);
}

void HttpFetcher::input(const TileId &tileId, TileFile type
                        , unsigned int revision
                        , const InputCallback &cb
                        , const IStream::pointer *notFound) const
{
    std::make_shared<AsyncFetcher>
        (joinUrl(rootUrl_, remotePath(tileId, type, revision))
         , contentType(type), options_, cb, notFound)->run();
}

} } } // namespace vtslibs::vts::driver
