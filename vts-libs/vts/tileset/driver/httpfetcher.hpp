#ifndef vtslibs_vts_tileset_driver_httpfetcher_hpp_included_
#define vtslibs_vts_tileset_driver_httpfetcher_hpp_included_

#include "../driver.hpp"
#include "../../options.hpp"

namespace vtslibs { namespace vts { namespace driver {

class HttpFetcher {
public:
    HttpFetcher(const std::string &rootUrl, const OpenOptions &options);

    IStream::pointer input(File type, bool noSuchFile = true) const;

    IStream::pointer input(const TileId &tileId, TileFile type
                           , unsigned int revision
                           , bool noSuchFile = true) const;

private:
    const std::string rootUrl_;
    OpenOptions options_;
};

} } } // namespace vtslibs::vts::driver

#endif // vtslibs_vts_tileset_driver_httpfetcher_hpp_included_
