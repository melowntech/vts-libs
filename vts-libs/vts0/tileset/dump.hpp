#ifndef vtslibs_vts0_tileset_dump_hpp_included_
#define vtslibs_vts0_tileset_dump_hpp_included_

#include <boost/filesystem/path.hpp>

#include "../tileindex.hpp"

namespace vtslibs { namespace vts0 {

const char* getDumpDir();

void dumpTileIndex(const char *root, const boost::filesystem::path &name
                   , const TileIndex &index);


} } // namespace vtslibs::vts0

#endif // vtslibs_vts0_tileset_dump_hpp_included_
