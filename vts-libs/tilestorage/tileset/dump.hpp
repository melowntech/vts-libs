#ifndef vadstena_libs_tilestorage_tileset_dump_hpp_included_
#define vadstena_libs_tilestorage_tileset_dump_hpp_included_

#include <boost/filesystem/path.hpp>

#include "../tileindex.hpp"

namespace vadstena { namespace tilestorage {

const char* getDumpDir();

void dumpTileIndex(const char *root, const fs::path &name
                   , const TileIndex &index);


} } // namespace vadstena::tilestorage

#endif // vadstena_libs_tilestorage_tileset_dump_hpp_included_
