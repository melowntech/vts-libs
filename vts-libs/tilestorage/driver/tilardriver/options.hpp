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
#ifndef vtslibs_tilestorage_driver_tilardriver_options_hpp_included_
#define vtslibs_tilestorage_driver_tilardriver_options_hpp_included_

#include <set>
#include <map>

#include <boost/optional.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/uuid/uuid.hpp>

#include "../../properties.hpp"
#include "../../driver.hpp"
#include "../../../storage/tilar.hpp"
#include "../../tileop.hpp"

namespace vtslibs { namespace tilestorage { namespace tilardriver {

using storage::Tilar;

struct Options {
    /** Tile size at LOD=0.
     */
    long baseTileSize;

    /** Tile alignment. No tile exists that contains this point inside.
     */
    Alignment alignment;

    /** Binary order of magnitude of data stored in the individial tile
     *  archives (each archive has square grid of
     *  (2^binaryOrder_)*(2^binaryOrder_) tiles.
     *
     * This information maps directly to LOD-shift (tile space of tiles at
     * any LOD are stored in space of "super" tiles at (LOD - binaryOrder_)).
     */
    std::uint8_t binaryOrder;

    /** UUID of storage. Generated automatically on creation. Passed to
     *  tilar file create/check.
     */
    boost::uuids::uuid uuid;

    /** Tile mask applied to tile index to get index inside archive.
     */
    long tileMask;

    /** Tilar options derived from the above for tiles.
     */
    Tilar::Options tilar(unsigned int filesPerTile) const;

    Options(const Driver::CreateProperties &properties);
    Options(const Driver::CreateProperties &properties, bool);

    struct Index {
        tilestorage::Index archive;
        Tilar::FileIndex file;
    };

    /** Converts tileId into index of tilar file in the super grid and a file
     * index inside this archive.
     */
    Index index(const TileId &tileId, int type) const;
};

// inlines

inline Tilar::Options Options::tilar(unsigned int filesPerTile) const
{
    return { binaryOrder, filesPerTile, uuid };
}

inline Options::Index Options::index(const TileId &tileId, int type) const
{
    // index of tile from alignment
    auto i(tileIndex(alignment, baseTileSize, tileId));
    return {
        tilestorage::Index(i.lod
                           , i.easting >> binaryOrder
                           , i.northing >> binaryOrder)
        , Tilar::FileIndex(i.easting & tileMask
                           , i.northing & tileMask
                           , type)
    };
}

} } } // namespace vtslibs::tilestorage::tilardriver

#endif // vtslibs_tilestorage_driver_tilardriver_options_hpp_included_
