/**
 * \file geometry_core.hpp
 * \author Vaclav Blazek <vaclav.blazek@citationtech.net>
 *
 * Tile storage manipulation.
 *
 * NB: tile set is specified by simple URI: TYPE:LOCATION where:
 *     TYPE     is type of backing storage (i.e. access driver to use);
 *              defaults to "flat"
 *     LOCATION is type-specific location of storage (e.g. root directory for
 *              filesystem based backing)
 */

#ifndef vadstena_libs_tilestorage_types_hpp_included_
#define vadstena_libs_tilestorage_types_hpp_included_

#include <opencv2/core/core.hpp>

#include "./basetypes.hpp"
#include "./metatile.hpp"

namespace vadstena { namespace tilestorage {

/** A tile: mesh + atlas.
 */
struct Tile {
    Mesh mesh;
    Atlas atlas;
    MetaNode metanode;

    typedef std::vector<Tile> list;

    Tile() {}
    Tile(const Mesh &mesh, const Atlas &atlas, const MetaNode &metanode)
        : mesh(mesh), atlas(atlas), metanode(metanode)
    {}
};

} } // namespace vadstena::tilestorage

#endif // vadstena_libs_tilestorage_types_hpp_included_
