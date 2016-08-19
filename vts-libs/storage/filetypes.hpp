#ifndef vadstena_libs_storage_filetypes_hpp_included_
#define vadstena_libs_storage_filetypes_hpp_included_

namespace vadstena { namespace storage {

enum class TileFile {
    // 3D interface
    meta, mesh, atlas, navtile

   // 2D interface
   , meta2d, mask, ortho, credits
};

enum class File { config, tileIndex, extraConfig, registry };

} } // namespace vadstena::storage

#endif // vadstena_libs_storage_filetypes_hpp_included_

