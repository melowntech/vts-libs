#include <algorithm>
#include <iterator>

#include <boost/format.hpp>
#include <boost/iostreams/device/array.hpp>

#include <curl/curl.h>

#include "../../../storage/error.hpp"
#include "../../tileop.hpp"
#include "./httpfetcher.hpp"

namespace vadstena { namespace vts { namespace driver {

namespace {
std::streamsize IOBufferSize(1 << 16);
typedef std::vector<char> Buffer;

/** Let the CURL machinery initialize here
 */
class CurlInitializer {
public:
    ~CurlInitializer() { ::curl_global_cleanup(); }

private:
    CurlInitializer() {
        if (::curl_global_init(CURL_GLOBAL_ALL)) {
            storage::IOError e("Global CURL initialization failed!");
            std::cerr << e.what() << std::endl;
            throw e;
        }
    };

    static CurlInitializer instance_;
};

CurlInitializer CurlInitializer::instance_;

std::shared_ptr< ::CURL> createCurl()
{
    auto c(::curl_easy_init());
    if (!c) {
        LOGTHROW(err2, storage::IOError)
            << "Failed to create CURL handle.";
    }

    // switch off SIGALARM
    ::curl_easy_setopt(c, CURLOPT_NOSIGNAL, 1L);

    // cache DNS query results during lifetime of this object
    ::curl_easy_setopt(c, CURLOPT_DNS_CACHE_TIMEOUT, -1L);

    return std::shared_ptr< ::CURL>
        (c, [](::CURL *c) { if (c) { ::curl_easy_cleanup(c); } });
}

CurlInitializer::CurlInitializer();

} // namespace

extern "C" {

size_t vts_drivers_detail_fetcher_write(char *ptr, size_t size, size_t nmemb
                                        , void *userdata)
{
    auto &buffer(*static_cast<Buffer*>(userdata));
    auto bytes(size * nmemb);
    std::copy(ptr, ptr + bytes, std::back_inserter(buffer));
    return bytes;
}

} // extern "C"

namespace {

// fake user-agent :)
const char *UserAgent
    ("Mozilla/5.0 (Windows NT 6.3; rv:36.0) Gecko/20100101 Firefox/36.0");

#define CHECK_CURL_STATUS(what)                                         \
    do {                                                                \
        auto res(what);                                                 \
        if (res != CURLE_OK) {                                          \
            LOGTHROW(err2, storage::IOError)                            \
                << "Failed to download tile from <"                     \
                << url << ">: <" << res << ", "                         \
                << ::curl_easy_strerror(res)                            \
                << ">.";                                                \
        }                                                               \
    } while (0)

#define SETOPT(name, value)                                     \
    CHECK_CURL_STATUS(::curl_easy_setopt(curl, name, value))

typedef std::tuple<long int, std::time_t> FetchResult;

std::tuple<long int, long int>
fetchUrl(::CURL *curl, const std::string &url, Buffer &buffer)
{
    LOG(info1) << "Fetching tile from <" << url << "> "
               << "(GET).";

    buffer.clear();

    // we are getting a resource
    SETOPT(CURLOPT_HTTPGET, 1L);
    SETOPT(CURLOPT_NOSIGNAL, 1L);

    // ask to keep last modified
    SETOPT(CURLOPT_FILETIME, 1L);

    // HTTP/1.1
    SETOPT(CURLOPT_HTTP_VERSION, CURL_HTTP_VERSION_1_1);
    // target url
    SETOPT(CURLOPT_URL, url.c_str());

    // set output function + userdata
    SETOPT(CURLOPT_WRITEFUNCTION, &vts_drivers_detail_fetcher_write);
    SETOPT(CURLOPT_WRITEDATA, &buffer);

    SETOPT(CURLOPT_USERAGENT, UserAgent);

    /// do the thing
    CHECK_CURL_STATUS(::curl_easy_perform(curl));

    // check status code:
    FetchResult fr(0, -1);
    auto &httpCode(std::get<0>(fr));
    auto &lastModified(std::get<1>(fr));

    CHECK_CURL_STATUS(::curl_easy_getinfo
                      (curl, CURLINFO_RESPONSE_CODE, &httpCode));

    CHECK_CURL_STATUS(::curl_easy_getinfo
                      (curl, CURLINFO_FILETIME, &lastModified));

    return fr;
}

inline ::CURL* handle(const std::shared_ptr<void> &handle)
{
    return static_cast< ::CURL*>(handle.get());
}

class BufferIStream : public IStream {
public:
    BufferIStream(Buffer &&buffer, const char *contentType
                  , const std::string &name, std::time_t lastModified)
        : IStream(contentType)
        , buffer_(std::move(buffer)), name_(name)
        , sb_(boost::iostreams::array_source(buffer_.data(), buffer_.size())
              , IOBufferSize, IOBufferSize)
        , stream_(&sb_)
    {
        stat_.lastModified = lastModified;
        stat_.size = buffer_.size();
    }

private:
    virtual std::istream& get() { return stream_; }
    virtual FileStat stat_impl() const { return stat_; }
    virtual void close() {};
    virtual std::string name() const { return name_; }

    Buffer buffer_;
    FileStat stat_;
    std::string name_;

    boost::iostreams::stream_buffer<boost::iostreams::array_source> sb_;
    std::istream stream_;
};


const std::string ConfigName("tileset.conf");
const std::string ExtraConfigName("extra.conf");
const std::string TileIndexName("tileset.index");

const std::string filePath(File type)
{
    switch (type) {
    case File::config: return ConfigName;
    case File::extraConfig: return ExtraConfigName;
    case File::tileIndex: return TileIndexName;
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

IStream::pointer fetchAsStream(::CURL *handle
                               , const std::string rootUrl
                               , const std::string &filename
                               , const char *contentType
                               , const HttpFetcher::Options &options)
{
    std::string url(rootUrl + "/" + filename);

    // TODO: make robust

    Buffer buffer;
    long int httpCode;
    std::time_t lastModified;

    auto tryFetch([&]() -> IStream::pointer
    {
        std::tie(httpCode, lastModified) = fetchUrl(handle, url, buffer);

        if (httpCode == 404) {
            LOGTHROW(err2, storage::NoSuchFile)
                << "File at URL <" << url << "> doesn't exist.";
        }

        if (httpCode != 200) {
            LOGTHROW(err2, storage::IOError)
                << "Failed to download tile data from <"
                << url << ">: Unexpected HTTP status code: <"
                << httpCode << ">.";
        }

        return std::make_shared<BufferIStream>
            (std::move(buffer), contentType, url, lastModified);
    });

    for (auto tries(options.tries); tries > 0; (tries > 0) ? --tries : 0) {
        try {
            return tryFetch();
        } catch (const std::exception &e) {
            LOG(warn2) << "Failed to fetch file from <" << url
                       << ">; retrying in a while.";
            ::sleep(1);
        }
    }
    return tryFetch();
}

} // namespace

HttpFetcher::HttpFetcher(const std::string &rootUrl
                         , const Options &options)
    : rootUrl_(rootUrl), options_(options), handle_(createCurl())
{}

IStream::pointer HttpFetcher::input(File type)
    const
{
    return fetchAsStream(handle(handle_), rootUrl_, filePath(type)
                         , contentType(type), options_);
}

IStream::pointer HttpFetcher::input(const TileId &tileId, TileFile type
                                    , unsigned int revision)
    const
{
    return fetchAsStream(handle(handle_), rootUrl_
                         , remotePath(tileId, type, revision)
                         , contentType(type), options_);
}

} } } // namespace vadstena::vts::driver
