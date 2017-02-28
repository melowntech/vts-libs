#ifndef vtslibs_vts0_driver_tilardriver_options_hpp_included_
#define vtslibs_vts0_driver_tilardriver_options_hpp_included_

#include <set>
#include <map>

#include <boost/optional.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/uuid/uuid.hpp>

#include "../../../storage/tilar.hpp"

#include "../../properties.hpp"
#include "../../driver.hpp"
#include "../../tileop.hpp"

namespace vtslibs { namespace vts0 { namespace tilardriver {

using storage::Tilar;

struct Options {
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
        vts0::TileId archive;
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

inline Options::Index Options::index(const TileId &i, int type) const
{
    return {
        vts0::TileId(i.lod, i.x >> binaryOrder, i.y >> binaryOrder)
       , Tilar::FileIndex(i.x & tileMask, i.y & tileMask, type)
    };
}

} } } // namespace vtslibs::vts0::tilardriver

#endif // vtslibs_vts0_driver_tilardriver_options_hpp_included_
