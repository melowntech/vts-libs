#ifndef vtslibs_tilestorage_tileset_dump_hpp_included_
#define vtslibs_tilestorage_tileset_dump_hpp_included_

#include <boost/filesystem/path.hpp>

#include "../tileindex.hpp"

namespace vtslibs { namespace tilestorage {

const char* getDumpDir();

void dumpTileIndex(const char *root, const fs::path &name
                   , const TileIndex &index);


} } // namespace vtslibs::tilestorage

#endif // vtslibs_tilestorage_tileset_dump_hpp_included_
