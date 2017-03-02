#ifndef vtslibs_vts_opencv_texture_hpp
#define vtslibs_vts_opencv_texture_hpp

#include <opencv2/core/core.hpp>

#include "../mesh.hpp"

namespace vtslibs { namespace vts { namespace opencv {

inline math::Point2d denormalize(const math::Point2d &p
                                 , const cv::Size &texSize)
{
    return { (p(0) * texSize.width), ((1.0 - p(1)) * texSize.height) };
}

inline math::Points2d denormalize(const math::Points2d &ps
                                  , const cv::Size &texSize)
{
    math::Points2d out;
    for (const auto &p : ps) {
        out.push_back(denormalize(p, texSize));
    }
    return out;
}

void rasterizeMask(cv::Mat &mask, const Faces &faces
                   , const math::Points2d &tc);

/** Rasterizes triangle mesh into mask. Each pixel is dilated by one pixel to
 *  the left and top (because it is automatically dilated to the opposite side
 *  by scan convert rasterization algo.
 */
void rasterizeMaskLegacy(cv::Mat &mask, const Faces &faces
                         , const math::Points2d &tc);

} } } // namespace vtslibs::vts::opencv

#endif // vtslibs_vts_opencv_texture_hpp
