#ifndef vadstena_libs_tilestorage_driver_tilardriver_options_hpp_included_
#define vadstena_libs_tilestorage_driver_tilardriver_options_hpp_included_

#include <set>
#include <map>

#include <boost/optional.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/uuid/uuid.hpp>

#include "../../properties.hpp"
#include "../../driver.hpp"
#include "../../../storage/tilar.hpp"
#include "../../tileop.hpp"

namespace vadstena { namespace tilestorage { namespace tilardriver {

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

} } } // namespace vadstena::tilestorage::tilardriver

#endif // vadstena_libs_tilestorage_driver_tilardriver_options_hpp_included_
