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
#include "./properties.hpp"

namespace vtslibs { namespace vts {

class Driver;
namespace tileset { class Index; }

using storage::IStream;
using storage::File;
using storage::TileFile;
using storage::FileStat;
using storage::Resources;
using storage::NullWhenNotFound_t;
using storage::NullWhenNotFound;

class Delivery : boost::noncopyable {
public:
    typedef std::shared_ptr<Delivery> pointer;

    // named ctor
    static pointer open(const boost::filesystem::path &root);

    IStream::pointer input(File type) const;

    IStream::pointer input(File type, const NullWhenNotFound_t&) const;

    IStream::pointer input(const TileId &tileId, TileFile type
                           , FileFlavor flavor) const;

    IStream::pointer input(const TileId &tileId, TileFile type
                           , FileFlavor flavor
                           , const NullWhenNotFound_t&) const;

    FileStat stat(File type) const;

    FileStat stat(const TileId &tileId, TileFile type) const;

    IStream::pointer input(const std::string &name) const;

    IStream::pointer input(const std::string &name
                           , const NullWhenNotFound_t&) const;

    FileStat stat(const std::string &name) const;

    Resources resources() const;

    bool externallyChanged() const;

    std::time_t lastModified() const;

    MapConfig mapConfig(bool includeExtra = true) const;

    MeshTilesConfig meshTilesConfig(bool includeExtra = true) const;

private:
    struct AccessToken {};

    std::shared_ptr<Driver> driver_;
    const FullTileSetProperties properties_;
    std::shared_ptr<tileset::Index> index_;

public:
    /** Opens storage.
     */
    Delivery(AccessToken, const boost::filesystem::path &root);
};

} } // namespace vtslibs::vts

#endif // vtslibs_vts_tileset_delivery_hpp_included_
