/**
 * Copyright (c) 2017 Melown Technologies SE
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * *  Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
/**
 * \file storage/tilar.hpp
 * \author Vaclav Blazek <vaclav.blazek@citationtech.net>
 *
 * Tile Archive handler.
 */

#ifndef vtslibs_storage_tilar_io_hpp_included_
#define vtslibs_storage_tilar_io_hpp_included_

#include "utility/enum-io.hpp"
#include "utility/time.hpp"

#include <iostream>
#include <boost/uuid/uuid_io.hpp>

namespace vtslibs { namespace storage {

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

} } // namespace vtslibs::storage

#endif // vtslibs_storage_tilar_io_hpp_included_
