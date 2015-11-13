#ifndef vadstena_libs_vts_tilesource_hpp_included_
#define vadstena_libs_vts_tilesource_hpp_included_

#include "../storage/streams.hpp"

#include "./metatile.hpp"

namespace vadstena { namespace vts {

using storage::IStream;

/** Tile source. Can be used to pump tile between tilesets.
 */
struct TileSource {
    MetaNode metanode;
    bool watertight;

    IStream::pointer mesh;
    IStream::pointer atlas;
    IStream::pointer navtile;

    TileSource() : watertight(false) {}
    TileSource(const MetaNode &metanode, bool watertight)
        : metanode(metanode), watertight(watertight)
    {}
};

} } // namespace vadstena::vts

#endif // vadstena_libs_vts_tilesource_hpp_included_
