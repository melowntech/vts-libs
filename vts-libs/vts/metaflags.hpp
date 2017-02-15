#ifndef vtslibs_vts_metaflags_hpp_included_
#define vtslibs_vts_metaflags_hpp_included_

#include <iostream>

#include "./metatile.hpp"

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
