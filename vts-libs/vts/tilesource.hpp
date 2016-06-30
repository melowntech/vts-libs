#ifndef vadstena_libs_vts_tilesource_hpp_included_
#define vadstena_libs_vts_tilesource_hpp_included_

#include "../storage/streams.hpp"

#include "./metatile.hpp"
#include "./tileindex.hpp"

namespace vadstena { namespace vts {

using storage::IStream;

/** Tile source. Can be used to pump tile between tilesets.
 */
struct TileSource {
    MetaNode metanode;
    TileIndex::Flag::value_type extraFlags;

    IStream::pointer mesh;
    IStream::pointer atlas;
    IStream::pointer navtile;

    TileSource() : extraFlags(0) {}
    TileSource(const MetaNode &metanode
               , TileIndex::Flag::value_type extraFlags)
        : metanode(metanode), extraFlags(extraFlags)
    {}
};

} } // namespace vadstena::vts

#endif // vadstena_libs_vts_tilesource_hpp_included_
