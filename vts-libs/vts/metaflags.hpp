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
#ifndef vtslibs_vts_metaflags_hpp_included_
#define vtslibs_vts_metaflags_hpp_included_

#include <iostream>

#include "metatile.hpp"

namespace vtslibs { namespace vts {

struct MetaFlags {
    MetaNode::Flag::value_type value;
    const MetaNode *node;

    MetaFlags(MetaNode::Flag::value_type value = 0)
        : value(value) {}

    MetaFlags(const MetaNode &node)
        : value(node.flags()) {}

    typedef std::pair<MetaNode::Flag::value_type, const char*> MetaFlag;
    static std::vector<MetaFlag> mapping;
};

template <typename Func>
void forEachFlag(const MetaFlags &f, const Func &callback)
{
    std::ostringstream os;

    for (const auto &flag : MetaFlags::mapping) {
        if (f.value & flag.first) {
            os.str("");
            os << flag.second;
            callback(os.str());
        }
    }
}

template<typename CharT, typename Traits>
inline std::basic_ostream<CharT, Traits>&
operator<<(std::basic_ostream<CharT, Traits> &os, const MetaFlags &f)
{
    const char *prefix("");
    for (const auto &flag : MetaFlags::mapping) {
        if (f.value & flag.first) {
            os << prefix << flag.second;
            prefix = ",";
        }
    }
    return os;
}

template<typename CharT, typename Traits>
inline std::basic_istream<CharT, Traits>&
operator>>(std::basic_istream<CharT, Traits> &is, MetaFlags &f)
{
    f.value = 0;
    std::string token;
    while (std::getline(is, token, ',')) {
        bool found(false);
        for (const auto &flag : MetaFlags::mapping) {
            if (token == flag.second) {
                f.value |= flag.first;
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

#endif // vtslibs_vts_metaflags_hpp_included_
