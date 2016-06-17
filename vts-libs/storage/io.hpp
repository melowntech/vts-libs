#ifndef vadstena_libs_storage_io_hpp_included_
#define vadstena_libs_storage_io_hpp_included_

#include "utility/enum-io.hpp"

#include "./filetypes.hpp"
#include "./resources.hpp"

namespace vadstena { namespace storage {

UTILITY_GENERATE_ENUM_IO(TileFile,
                         ((meta))
                         ((mesh))
                         ((atlas))
                         ((navtile))
                         ((meta2d))
                         ((mask))
                         ((ortho))
                         ((credits))
                         )

UTILITY_GENERATE_ENUM_IO(File,
                         ((config))
                         ((tileIndex))
                         )

template<typename CharT, typename Traits>
inline std::basic_ostream<CharT, Traits>&
operator<<(std::basic_ostream<CharT, Traits> &os
           , const Resources &r)
{
    return os << "{openFiles=" << r.openFiles
              << ", memory=" << r.memory << '}';
}

} } // namespace vadstena::storage

#endif // vadstena_libs_storage_io_hpp_included_

