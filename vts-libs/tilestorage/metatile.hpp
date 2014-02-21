#ifndef vadstena_libs_tilestorage_metatile_hpp_included_
#define vadstena_libs_tilestorage_metatile_hpp_included_

#include <iosfwd>
#include <functional>

#include "./types.hpp"
#include "../metatile.hpp"

namespace vadstena { namespace tilestorage {

typedef std::function<void(const TileId &tileId
                           , const MetaNode &node)> MetaNodeLoader;

void loadMetatile(std::istream &f, long baseTileSize, const TileId &tileId
                  , const MetaNodeLoader &loader);

} } // namespace vadstena::tilestorage

#endif // vadstena_libs_tilestorage_metatile_hpp_included_
