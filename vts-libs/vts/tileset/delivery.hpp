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
#ifndef vtslibs_vts_tileset_delivery_hpp_included_
#define vtslibs_vts_tileset_delivery_hpp_included_

#include <memory>

#include <boost/noncopyable.hpp>

#include "../../storage/streams.hpp"
#include "../../storage/resources.hpp"
#include "../mapconfig.hpp"
#include "properties.hpp"

#include "driver/streams.hpp"

namespace vtslibs { namespace vts {

class Driver;
namespace tileset { class Index; }

class Delivery : boost::noncopyable {
public:
    typedef std::shared_ptr<Delivery> pointer;

    // named ctor
    static pointer open(const boost::filesystem::path &root
                        , const OpenOptions &openOptions = OpenOptions());

    static pointer open(std::shared_ptr<Driver> driver);

    IStream::pointer input(File type) const;

    IStream::pointer input(File type, const NullWhenNotFound_t&) const;

    IStream::pointer input(const TileId &tileId, TileFile type
                           , FileFlavor flavor) const;

    IStream::pointer input(const TileId &tileId, TileFile type
                           , FileFlavor flavor
                           , const NullWhenNotFound_t&) const;

    void input(const TileId &tileId, TileFile type, FileFlavor flavor
               , const InputCallback &cb) const;

    FileStat stat(File type) const;

    FileStat stat(const TileId &tileId, TileFile type) const;

    void stat(const TileId &tileId, TileFile type
              , const StatCallback &cb) const;

    IStream::pointer input(const std::string &name) const;

    IStream::pointer input(const std::string &name
                           , const NullWhenNotFound_t&) const;

    FileStat stat(const std::string &name) const;

    Resources resources() const;

    bool externallyChanged() const;

    std::time_t lastModified() const;

    MapConfig mapConfig(bool includeExtra = true) const;

    MeshTilesConfig meshTilesConfig(bool includeExtra = true) const;

    /** Access underlying driver. Use wisely.
     */
    std::shared_ptr<Driver> driver() const { return driver_; }

private:
    struct AccessToken {};

    std::shared_ptr<Driver> driver_;
    const FullTileSetProperties properties_;
    std::shared_ptr<tileset::Index> index_;

public:
    /** Opens storage.
     */
    Delivery(AccessToken, const boost::filesystem::path &root
             , const OpenOptions &openOptions = OpenOptions());

    Delivery(AccessToken, std::shared_ptr<Driver> driver);
};

} } // namespace vtslibs::vts

#endif // vtslibs_vts_tileset_delivery_hpp_included_
