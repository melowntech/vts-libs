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
#include "./httpfetcher.hpp"

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
        default: throw "Unexpected TileFile value. Go fix your program.";
        }
        throw;
    }());

    return str(boost::format("%d-%d-%d%s?%d")
               % tileId.lod % tileId.x % tileId.y % ext % revision);
}

http::OnDemandClient sharedClient(4);

IStream::pointer fetchAsStream(const std::string rootUrl
                               , const std::string &filename
                               , const char *contentType
                               , const OpenOptions &options
                               , bool noSuchFile)
{
    const auto &fetcher(sharedClient.fetcher());

    const std::string url(rootUrl + "/" + filename);

    auto tryFetch([&]() -> IStream::pointer
    {
        auto q(fetcher.perform
               (utility::ResourceFetcher::Query(url)
                .timeout(options.ioWait())));

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

    for (auto tries(options.ioRetries()); tries; (tries > 0) ? --tries : 0) {
        try {
            return tryFetch();
        } catch (const storage::IOError &e) {
            LOG(warn2) << "Failed to fetch file from <" << url
                       << ">; retrying in a while.";
            ::sleep(1);
        }
    }
    return tryFetch();
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
    return fetchAsStream(rootUrl_, filePath(type)
                         , contentType(type), options_
                         , noSuchFile);
}

IStream::pointer HttpFetcher::input(const TileId &tileId, TileFile type
                                    , unsigned int revision, bool noSuchFile)
    const
{
    return fetchAsStream(rootUrl_, remotePath(tileId, type, revision)
                         , contentType(type), options_
                         , noSuchFile);
}

} } } // namespace vtslibs::vts::driver
