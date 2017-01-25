#ifndef vadstena_libs_registry_io_hpp_included_
#define vadstena_libs_registry_io_hpp_included_

#include <iostream>

#include "../registry.hpp"

namespace vadstena { namespace registry {

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


} } // namespace vadstena::registry

#endif // vadstena_libs_registry_io_hpp_included_
