#ifndef vadstena_libs_vts0_tileindex_io_hpp_included_
#define vadstena_libs_vts0_tileindex_io_hpp_included_

#include <iostream>

#include "./tileindex.hpp"

namespace vadstena { namespace vts0 {

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

} } // namespace vadstena::vts0

#endif // vadstena_libs_vts0_tileindex_io_hpp_included_
