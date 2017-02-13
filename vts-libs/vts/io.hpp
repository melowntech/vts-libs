#ifndef vadstena_libs_vts_io_hpp_included_
#define vadstena_libs_vts_io_hpp_included_

#include <iostream>

#include "utility/streams.hpp"

#include "../storage/io.hpp"
#include "./basetypes.hpp"
#include "./geomextents.hpp"

namespace vadstena { namespace vts {

template<typename CharT, typename Traits>
inline std::basic_istream<CharT, Traits>&
operator>>(std::basic_istream<CharT, Traits> &is
           , LodTileRange &ltr)
{
    return is >> ltr.lod >> utility::expect('/') >> ltr.range;
}

template<typename CharT, typename Traits>
inline std::basic_ostream<CharT, Traits>&
operator<<(std::basic_ostream<CharT, Traits> &os
           , const LodTileRange &ltr)
{
    return os << ltr.lod << '/' << ltr.range;
}

template<typename CharT, typename Traits>
inline std::basic_ostream<CharT, Traits>&
operator<<(std::basic_ostream<CharT, Traits> &os
           , const Ranges &ranges)
{
    return os << ranges.lodRange() << '/' << ranges.tileRange();
}

template<typename CharT, typename Traits>
inline std::basic_ostream<CharT, Traits>&
operator<<(std::basic_ostream<CharT, Traits> &os
           , const GeomExtents &ge)
{
    return os << ge.z << ',' << ge.surrogate;
}

} } // namespace vadstena::vts

#endif // vadstena_libs_vts_io_hpp_included_
