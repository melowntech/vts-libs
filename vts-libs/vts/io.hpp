#ifndef vadstena_libs_vts_io_hpp_included_
#define vadstena_libs_vts_io_hpp_included_

#include <iostream>
#include <typeinfo>

#include <boost/spirit/include/qi.hpp>
#include <boost/spirit/include/qi_match.hpp>
#include <boost/spirit/include/qi_match_auto.hpp>

#include "./basetypes.hpp"

namespace vadstena { namespace vts {

template<typename CharT, typename Traits>
inline std::basic_ostream<CharT, Traits>&
operator<<(std::basic_ostream<CharT, Traits> &os, const TileId &id)
{
    return os << id.lod << '-' << id.x << '-' << id.y;
}

template<typename CharT, typename Traits>
inline std::basic_ostream<CharT, Traits>&
dump(std::basic_ostream<CharT, Traits> &os
     , const TileId &tid
     , const std::string &prefix = std::string())
{
    os << prefix << "lod = " << (unsigned int)(tid.lod) << '\n'
       << prefix << "x = " << tid.x << '\n'
       << prefix << "y = " << tid.y << '\n'
        ;

    return os;
}

} } // namespace vadstena::vts

#endif // vadstena_libs_vts_io_hpp_included_
