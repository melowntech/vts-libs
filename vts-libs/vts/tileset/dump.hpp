#ifndef vadstena_libs_vts_tileset_dump_hpp_included_
#define vadstena_libs_vts_tileset_dump_hpp_included_

#include <boost/filesystem/path.hpp>

#include "../tileindex.hpp"

namespace vadstena { namespace vts {

const char* getDumpDir();

void dumpTileIndex(const char *root, const fs::path &name
                   , const TileIndex &index);


} } // namespace vadstena::vts

#endif // vadstena_libs_vts_tileset_dump_hpp_included_
