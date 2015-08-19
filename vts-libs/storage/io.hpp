#ifndef vadstena_libs_storage_io_hpp_included_
#define vadstena_libs_storage_io_hpp_included_

#include "utility/enum-io.hpp"

#include "./filetypes.hpp"

namespace vadstena { namespace storage {

UTILITY_GENERATE_ENUM_IO(TileFile,
                         ((meta))
                         ((mesh))
                         ((atlas))
                         )

UTILITY_GENERATE_ENUM_IO(File,
                         ((config))
                         ((tileIndex))
                         )

} } // namespace vadstena::storage

#endif // vadstena_libs_storage_io_hpp_included_

