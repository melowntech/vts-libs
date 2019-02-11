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
#ifndef vtslibs_vts_tileflags_hpp_included_
#define vtslibs_vts_tileflags_hpp_included_

#include <iostream>
#include <sstream>

#include "tileindex.hpp"

namespace vtslibs { namespace vts {

struct TileFlags {
    typedef TileIndex::Flag TiFlag;
    TiFlag::value_type value;

    TileFlags(TileIndex::Flag::value_type value = 0) : value(value) {}

    operator TileIndex::Flag::value_type() const { return value; }

    typedef std::pair<TiFlag::value_type, TiFlag::value_type> Match;
    typedef std::pair<Match, const char*> TileFlag;
    static std::vector<TileFlag> mapping;
};

template <typename Func>
void forEachFlag(const TileFlags &f, const Func &callback)
{
    std::ostringstream os;

    for (const auto &flag : TileFlags::mapping) {
        const auto &match(flag.first);
        if ((f.value & match.first) == match.second) {
            os.str("");
            os << flag.second;
            callback(os.str());
        }
    }

    if (auto reference = TileIndex::Flag::getReference(f.value)) {
        os.str("");
        os << "reference=" << reference;
        callback(os.str());
    }
}

template<typename CharT, typename Traits>
inline std::basic_ostream<CharT, Traits>&
operator<<(std::basic_ostream<CharT, Traits> &os, const TileFlags &f)
{
    const char *prefix("");
    for (const auto &flag : TileFlags::mapping) {
        const auto &match(flag.first);
        if ((f.value & match.first) == match.second) {
            os << prefix << flag.second;
            prefix = ",";
        }
    }
    if (auto reference = TileIndex::Flag::getReference(f.value)) {
        os << prefix << "reference=" << reference;
    }
    return os;
}

template<typename CharT, typename Traits>
inline std::basic_istream<CharT, Traits>&
operator>>(std::basic_istream<CharT, Traits> &is, TileFlags &f)
{
    f.value = 0;
    std::string token;
    while (std::getline(is, token, ',')) {
        bool found(false);
        for (const auto &flag : TileFlags::mapping) {
            if (token == flag.second) {
                f.value |= flag.first.second;
                found = true;
                break;
            }
        }

        if (!found) {
            // invalid value
            is.setstate(std::ios::failbit);
            return is;
        }
    }

    // get rid of eof bit
    is.clear(std::ios_base::eofbit);

    return is;
}

} } // namespace vtslibs::vts

#endif // vtslibs_vts_tileflags_hpp_included_
