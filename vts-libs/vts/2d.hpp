#ifndef vadstena_libs_vts_2d_hpp_included_
#define vadstena_libs_vts_2d_hpp_included_

#include <iostream>

#include <boost/gil/gil_all.hpp>

#include "./types2d.hpp"
#include "./mesh.hpp"
#include "./tileindex.hpp"

namespace vadstena { namespace vts {

typedef boost::gil::gray8_image_t GrayImage;
typedef boost::gil::rgb8_image_t RgbImage;
typedef boost::gil::rgba8_image_t RgbaImage;

GrayImage mask2d(const Mesh::CoverageMask &coverageMask
                 , const std::vector<SubMesh::SurfaceReference>
                 &surfaceReferences
                 , bool singleSourced = false);

GrayImage mask2d(const MeshMask &mask, bool singleSourced = false);

GrayImage meta2d(const TileIndex &tileIndex, const TileId &tileId);

void saveCreditTile(std::ostream &out, const CreditTile &creditTile
                    , bool inlineCredits = true);

CreditTile loadCreditTile(std::istream &in, const boost::filesystem::path &path
                          = "unknown");

RgbaImage debugMask(const Mesh::CoverageMask &coverageMask
                    , const std::vector<SubMesh::SurfaceReference>
                    &surfaceReferences
                    , bool singleSourced = false);

RgbaImage debugMask(const MeshMask &mask, bool singleSourced = false);

RgbaImage emptyDebugMask();

// inlines

inline GrayImage mask2d(const MeshMask &mask, bool singleSourced) {
    return mask2d(mask.coverageMask, mask.surfaceReferences, singleSourced);
}

inline RgbaImage debugMask(const MeshMask &mask, bool singleSourced) {
    return debugMask(mask.coverageMask, mask.surfaceReferences, singleSourced);
}

} } // vadstena::vts

#endif // vadstena_libs_vts_2d_hpp_included_
