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
#ifndef vtslibs_vts_io_hpp_included_
#define vtslibs_vts_io_hpp_included_

#include <iostream>

#include "utility/streams.hpp"

#include "../storage/io.hpp"
#include "basetypes.hpp"
#include "basetypes3.hpp"
#include "geomextents.hpp"

namespace vtslibs { namespace vts {

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
    return os << ge.z << " (surrogate:" << ge.surrogate << ')';
}

template<typename CharT, typename Traits>
inline std::basic_ostream<CharT, Traits>&
operator<<(std::basic_ostream<CharT, Traits> &os
           , const TileId3 &tileId)
{
    return os << tileId.lod << '-' << tileId.x << '-' << tileId.y
              << '-' << tileId.z;
}

template<typename CharT, typename Traits>
inline std::basic_istream<CharT, Traits>&
operator>>(std::basic_istream<CharT, Traits> &is
           , TileId3 &tileId)
{
    return is >> tileId.lod
              >> utility::expect('-') >> tileId.x
              >> utility::expect('-') >> tileId.y
              >> utility::expect('-') >> tileId.z;
}

} } // namespace vtslibs::vts

#endif // vtslibs_vts_io_hpp_included_
