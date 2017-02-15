#ifndef vtslibs_storage_filetypes_hpp_included_
#define vtslibs_storage_filetypes_hpp_included_

namespace vtslibs { namespace storage {

enum class TileFile {
    // 3D interface
    meta, mesh, atlas, navtile

   // 2D interface
   , meta2d, mask, ortho, credits
};

enum class File { config, tileIndex, extraConfig, registry };

} } // namespace vtslibs::storage

#endif // vtslibs_storage_filetypes_hpp_included_

