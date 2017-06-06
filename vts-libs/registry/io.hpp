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
#ifndef vtslibs_registry_io_hpp_included_
#define vtslibs_registry_io_hpp_included_

#include <iostream>
#include <sstream>

#include "utility/streams.hpp"

#include "../registry.hpp"

namespace vtslibs { namespace registry {

template<typename CharT, typename Traits>
inline std::basic_ostream<CharT, Traits>&
operator<<(std::basic_ostream<CharT, Traits> &os, const Position &p)
{
    return os
        << p.type << ',' << p.position(0) << ',' << p.position(1)
        << ',' << p.heightMode << ',' << p.position(2)
        << ',' << p.orientation(0) << ',' << p.orientation(1)
        << ',' << p.orientation(2) << ',' << p.verticalExtent
        << ',' << p.verticalFov;
}

template<typename CharT, typename Traits>
inline std::basic_istream<CharT, Traits>&
operator>>(std::basic_istream<CharT, Traits> &is, Position &p)
{
    // we need to parse enumerations by getline since stream parser stops at
    // a whitespace, not a comma
    std::istringstream tmpis;
    std::string tmp;

    getline(is, tmp, ',');
    tmpis.str(tmp);
    tmpis >> p.type;

    is >> p.position(0)
       >> utility::expect(',') >> p.position(1)
       >> utility::expect(',');

    getline(is, tmp, ',');
    tmpis.str(tmp);
    tmpis >> p.heightMode;


    is >> p.position(2)
       >> utility::expect(',') >> p.orientation(0)
       >> utility::expect(',') >> p.orientation(1)
       >> utility::expect(',') >> p.orientation(2)
       >> utility::expect(',') >> p.verticalExtent
       >> utility::expect(',') >> p.verticalFov
        ;

    return is;
}


} } // namespace vtslibs::registry

#endif // vtslibs_registry_io_hpp_included_
