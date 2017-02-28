/**
 * \file vts/detai/extra.hpp
 * \author Vaclav Blazek <vaclav.blazek@melown.com>
 *
 * Extra stuff
 */

#ifndef vtslibs_vts_tileset_extra_hpp_included_
#define vtslibs_vts_tileset_extra_hpp_included_

namespace vtslibs { namespace vts {

void reencodeTileSet(const boost::filesystem::path &root
                     , const ReencodeOptions &options);

} } // namespace vtslibs::vts

#endif // vtslibs_vts_tileset_extra_hpp_included_
