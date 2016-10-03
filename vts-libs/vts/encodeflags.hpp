#ifndef vadstena_libs_vts_encodeflags_hpp_included_
#define vadstena_libs_vts_encodeflags_hpp_included_

#include <iostream>

#include "./options.hpp"

namespace vadstena { namespace vts {

struct EncodeFlags {
    CloneOptions::EncodeFlag::value_type value;
    bool reference;

    EncodeFlags(CloneOptions::EncodeFlag::value_type value = 0)
        : value(value) {}

    typedef std::pair<CloneOptions::EncodeFlag::value_type
                      , const char*> EncodeFlag;
    static std::vector<EncodeFlag> mapping;
};

template<typename CharT, typename Traits>
inline std::basic_ostream<CharT, Traits>&
operator<<(std::basic_ostream<CharT, Traits> &os, const EncodeFlags &f)
{
    const char *prefix("");
    for (const auto &flag : EncodeFlags::mapping) {
        if (f.value & flag.first) {
            os << prefix << flag.second;
            prefix = ",";
        }
    }
    return os;
}

template<typename CharT, typename Traits>
inline std::basic_istream<CharT, Traits>&
operator>>(std::basic_istream<CharT, Traits> &is, EncodeFlags &f)
{
    f.value = 0;
    std::string token;
    while (std::getline(is, token, ',')) {
        bool found(false);
        for (const auto &flag : EncodeFlags::mapping) {
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

} } // namespace vadstena::vts

#endif // vadstena_libs_vts_encodeflags_hpp_included_
