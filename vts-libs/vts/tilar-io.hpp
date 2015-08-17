/**
 * \file vts/tilar.hpp
 * \author Vaclav Blazek <vaclav.blazek@citationtech.net>
 *
 * Tile Archive handler.
 */

#ifndef vadstena_libs_vts_tilar_io_hpp_included_
#define vadstena_libs_vts_tilar_io_hpp_included_

#include "utility/enum-io.hpp"
#include "utility/time.hpp"

#include <iostream>
#include <boost/uuid/uuid_io.hpp>

namespace vadstena { namespace vts {

UTILITY_GENERATE_ENUM_IO(Tilar::CreateMode,
                         ((truncate))
                         ((failIfExists))
                         ((append))
                         ((appendOrTruncate))
                         )

UTILITY_GENERATE_ENUM_IO(Tilar::OpenMode,
                         ((readOnly))
                         ((readWrite))
                         )

template<typename CharT, typename Traits>
inline std::basic_ostream<CharT, Traits>&
dump(std::basic_ostream<CharT, Traits> &os
     , const Tilar::Options &o
     , const std::string &prefix = std::string())
{
    os << prefix << "binaryOrder = " << o.binaryOrder << '\n'
       << prefix << "filesPerTile = " << o.filesPerTile << '\n'
       << prefix << "uuid = " << o.uuid << '\n'
        ;

    return os;
}

template<typename CharT, typename Traits>
inline std::basic_ostream<CharT, Traits>&
operator<<(std::basic_ostream<CharT, Traits> &os
           , const Tilar::FileIndex &i)
{
    return os << '[' << i.col << ',' << i.row << ',' << i.type << ']';
}

template<typename CharT, typename Traits>
inline std::basic_ostream<CharT, Traits>&
dump(std::basic_ostream<CharT, Traits> &os
     , const Tilar::Info &i
     , const std::string &prefix = std::string())
{
    os << prefix << "offset = " << i.offset << '\n'
       << prefix << "previousOffset = " << i.previousOffset << '\n'
       << prefix << "overhead = " << i.overhead << '\n'
       << prefix << "modified = " << utility::formatDateTime(i.modified)
       << '\n';

    return os;
}

} } // namespace vadstena::vts

#endif // vadstena_libs_vts_tilar_io_hpp_included_
