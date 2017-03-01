#include "./texture.hpp"

namespace vtslibs { namespace vts { namespace opencv {

namespace {

constexpr int rasterizeFractionBits(8);
constexpr int rasterizeFractionMult(1 << rasterizeFractionBits);

}

/** Rasterizes triangle mesh into mask. Each pixel is dilated by one pixel to
 *  the left and top (because it is automatically dilated to the opposite side
 *  by scan convert rasterization algo.
 */
void rasterizeMask(cv::Mat &mask, const Faces &faces
                   , const math::Points2d &tc)
{
    const auto white(cv::Scalar(0xff));

    cv::Point poly[3];
    for (const auto &face : faces) {
        for (int i = 0; i < 3; ++i) {
            const auto &uv(tc[face(i)]);
            auto &p(poly[i]);
            // NB: we need to shift by a half pixel to map from grid (used by
            // this algo) to pixels (used by OpenCV)
            p.x = (uv(0) - 0.5) * rasterizeFractionMult;
            p.y = (uv(1) - 0.5) * rasterizeFractionMult;
        }

        int counts(3);
        const cv::Point *asPtr(poly);
        cv::fillConvexPoly(mask, poly, counts, white, 4
                           , rasterizeFractionBits);
        cv::polylines(mask, &asPtr, &counts, 1, true, white, 1, 4
                      , rasterizeFractionBits);
    }
}

} } } // namespace vtslibs::vts::opencv
