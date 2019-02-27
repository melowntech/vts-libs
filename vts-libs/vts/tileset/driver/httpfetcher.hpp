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

    void input(const TileId &tileId, TileFile type
               , unsigned int revision
               , const InputCallback &cb
               , const IStream::pointer *notFound) const;

private:
    const std::string rootUrl_;
    OpenOptions options_;
};

} } } // namespace vtslibs::vts::driver

#endif // vtslibs_vts_tileset_driver_httpfetcher_hpp_included_
