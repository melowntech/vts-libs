#include <opencv2/imgproc/imgproc.hpp>

#include "imgproc/scanconversion.hpp"

#include "./texture.hpp"

namespace vtslibs { namespace vts { namespace opencv {

namespace {

constexpr int rasterizeFractionBits(8);
constexpr int rasterizeFractionMult(1 << rasterizeFractionBits);

}

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

/** Rasterizes triangle mesh into mask. Each pixel is dilated by one pixel to
 *  the left and top (because it is automatically dilated to the opposite side
 *  by scan convert rasterization algo.
 */
void rasterizeMaskLegacy(cv::Mat &mask, const Faces &faces
                         , const math::Points2d &tc)
{
    const auto white(cv::Scalar(0xff));
    const cv::Rect bounds(0, 0, mask.cols, mask.rows);

    auto paint([&](int x, int y, int w, int h) -> void
    {
        cv::Rect r(x, y, w, h);
        cv::rectangle(mask, r & bounds, white, CV_FILLED, 4);
    });

    cv::Point3f tri[3];
    for (const auto &face : faces) {
        for (int i : { 0, 1, 2 }) {
            const auto &p(tc[face(i)]);
            tri[i] = { float(p(0)), float(p(1)), 0.f };
            paint(p(0), p(1), 1, 1);
        }

        // rasterize triangle
        std::vector<imgproc::Scanline> scanlines;
        imgproc::scanConvertTriangle(tri, 0, mask.rows, scanlines);

        for (const auto &sl : scanlines) {
            imgproc::processScanline(sl, 0, mask.cols
                                     , [&](int x, int y, float)
            {
                paint(x - 1, y - 1, 2, 2);
            });
        }
    }
}

void dilate(cv::Mat &mask, int distance)
{
    const auto se(cv::getStructuringElement(cv::MORPH_ELLIPSE
                                            , cv::Size(2 * distance + 1
                                                       , 2 * distance + 1)));
    cv::Mat tmp(mask.size(), mask.type());
    cv::dilate(mask, tmp, se);
    std::swap(mask, tmp);
}

} } } // namespace vtslibs::vts::opencv
