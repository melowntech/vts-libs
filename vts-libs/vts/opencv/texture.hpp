#ifndef vtslibs_vts_opencv_texture_hpp
#define vtslibs_vts_opencv_texture_hpp

#include <opencv2/core/core.hpp>

#include "imgproc/uvpack.hpp"

#include "../mesh.hpp"

namespace vtslibs { namespace vts { namespace opencv {

/**
 * source: grid in range (0, 1), flipped y
 * destination: pixel coordinates (-0.5, size - 0.5)
 */
template <typename SizeType>
math::Point2d denormalize(const math::Point2d &p
                                 , const SizeType &texSize)
{
    return { (p(0) * texSize.width) - 0.5
            , ((1.0 - p(1)) * texSize.height) - 0.5 };
}

template <typename SizeType>
math::Points2d denormalize(const math::Points2d &ps
                           , const SizeType &texSize)
{
    math::Points2d out;
    for (const auto &p : ps) {
        out.push_back(denormalize(p, texSize));
    }
    return out;
}

/**
 * destination: pixel coordinates (-0.5, size - 0.5)
 * source: grid in range (0, 1), flipped y
 */
template <typename SizeType>
math::Point2d normalize(const imgproc::UVCoord &uv
                        , const SizeType &texSize)
{
    return { (uv.x + 0.5) / texSize.width
            , 1.0 - (uv.y + 0.5) / texSize.height };
}

/**
 * destination: pixel coordinates (-0.5, size - 0.5)
 * source: grid in range (0, 1), flipped y
 */
template <typename SizeType>
math::Point2d normalize(const math::Point2 &uv
                        , const SizeType &texSize)
{
    return { (uv(0) + 0.5) / texSize.width
            , 1.0 - (uv(1) + 0.5) / texSize.height };
}

void rasterizeMask(cv::Mat &mask, const Faces &faces
                   , const math::Points2d &tc);

void rasterizeMask(cv::Mat &mask, const Faces &faces
                   , const math::Points2d &tc
                   , const std::set<int> &faceSubset);

/** Rasterizes triangle mesh into mask. Each pixel is dilated by one pixel to
 *  the left and top (because it is automatically dilated to the opposite side
 *  by scan convert rasterization algo.
 */
void rasterizeMaskLegacy(cv::Mat &mask, const Faces &faces
                         , const math::Points2d &tc);

/** Rasterizes triangle mesh into mask. Each pixel is dilated by one pixel to
 *  the left and top (because it is automatically dilated to the opposite side
 *  by scan convert rasterization algo.
 */
void rasterizeMaskLegacy(cv::Mat &mask, const Faces &faces
                         , const math::Points2d &tc
                         , const std::set<int> &faceSubset);

void dilate(cv::Mat &mask, int distance);

} } } // namespace vtslibs::vts::opencv

#endif // vtslibs_vts_opencv_texture_hpp
