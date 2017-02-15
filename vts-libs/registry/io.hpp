#ifndef vtslibs_registry_io_hpp_included_
#define vtslibs_registry_io_hpp_included_

#include <iostream>

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


} } // namespace vtslibs::registry

#endif // vtslibs_registry_io_hpp_included_
