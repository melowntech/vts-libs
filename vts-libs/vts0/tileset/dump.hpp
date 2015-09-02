#ifndef vadstena_libs_vts0_tileset_dump_hpp_included_
#define vadstena_libs_vts0_tileset_dump_hpp_included_

#include <boost/filesystem/path.hpp>

#include "../tileindex.hpp"

namespace vadstena { namespace vts0 {

const char* getDumpDir();

void dumpTileIndex(const char *root, const boost::filesystem::path &name
                   , const TileIndex &index);


} } // namespace vadstena::vts0

#endif // vadstena_libs_vts0_tileset_dump_hpp_included_
