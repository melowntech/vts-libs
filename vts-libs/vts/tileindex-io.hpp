#ifndef vadstena_libs_vts_tileindex_io_hpp_included_
#define vadstena_libs_vts_tileindex_io_hpp_included_

#include <iostream>

#include "./tileindex.hpp"

namespace vadstena { namespace vts {

template<typename CharT, typename Traits>
inline std::basic_ostream<CharT, Traits>&
operator<<(std::basic_ostream<CharT, Traits> &os
           , const TileIndex &i)
{
    os << "TileIndex:"
       << "\n    lods: " << i.lodRange()
       << "\n";

    return os;
}

} } // namespace vadstena::vts

#endif // vadstena_libs_vts_tileindex_io_hpp_included_
