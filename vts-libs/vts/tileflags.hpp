#ifndef vadstena_libs_vts_tileflags_hpp_included_
#define vadstena_libs_vts_tileflags_hpp_included_

#include <iostream>
#include <sstream>

#include "./tileindex.hpp"

namespace vadstena { namespace vts {

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

} } // namespace vadstena::vts

#endif // vadstena_libs_vts_tileflags_hpp_included_
