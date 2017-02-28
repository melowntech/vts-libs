#ifndef vtslibs_tilestorage_types_hpp_included_
#define vtslibs_tilestorage_types_hpp_included_

#include <opencv2/core/core.hpp>

#include "geometry/parse-obj.hpp"

#include "./basetypes.hpp"
#include "./typesfwd.hpp"

namespace vtslibs { namespace tilestorage {

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

} } // namespace vtslibs::tilestorage

#endif // vtslibs_tilestorage_types_hpp_included_
