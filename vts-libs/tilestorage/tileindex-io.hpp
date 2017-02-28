#ifndef vtslibs_tilestorage_tileindex_io_hpp_included_
#define vtslibs_tilestorage_tileindex_io_hpp_included_

#include <iostream>

#include <boost/spirit/include/qi.hpp>
#include <boost/spirit/include/qi_match.hpp>
#include <boost/spirit/include/qi_match_auto.hpp>

#include "./tileindex.hpp"

namespace vtslibs { namespace tilestorage {

template<typename CharT, typename Traits>
inline std::basic_ostream<CharT, Traits>&
operator<<(std::basic_ostream<CharT, Traits> &os
           , const TileIndex &i)
{
    os << "TileIndex:"
       << "\n    lods: " << i.lodRange()
       << "\n    extents: " << i.extents()
       << "\n";

    return os;
}

} } // namespace vtslibs::tilestorage

#endif // vtslibs_tilestorage_tileindex_io_hpp_included_
