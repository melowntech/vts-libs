#include "./2d.hpp"
#include "./qtree-rasterize.hpp"

namespace gil = boost::gil;

namespace vadstena { namespace vts {

MaskImage mask2d(const MeshMask &mask)
{
    typedef MaskImage::point_t Point;

    auto size(Mask2d::size());
    MaskImage out(size.width, size.height, gil::gray8_pixel_t(0x00), 0);

    auto outView(view(out));

    auto maskSize(Mask2d::maskSize());
    auto maskView(gil::subimage_view
                  (outView, 0, 0, maskSize.width, maskSize.height));
    rasterize(mask.coverageMask, maskView);

    // get iterators to flag and surface rows
    auto iflag(outView.row_begin(Mask2d::flagRow));
    auto isurfaceReference(outView.row_begin(Mask2d::surfaceRow));
    // submeshes start at 1
    ++iflag;
    ++isurfaceReference;
    for (const auto &sr : mask.surfaceReferences) {
        *iflag++ = Mask2d::Flag::submesh;
        *isurfaceReference++ = sr;
    }

    return out;
}

} } // vadstena::vts
