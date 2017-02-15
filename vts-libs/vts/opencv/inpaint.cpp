#include <opencv2/highgui/highgui.hpp>

#include "imgproc/scanconversion.hpp"
#include "imgproc/inpaint.hpp"

#include "../mesh.hpp"
#include "./atlas.hpp"

namespace vtslibs { namespace vts {

namespace {

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

/** Rasterizes triangle mesh into mask. Each pixel is dilated by one pixel to
 *  the left and top (because it is automatically dilated to the opposite side
 *  by scan convert rasterization algo.
 */
void rasterizeMask(cv::Mat &mask, const Faces &faces
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

} // namespace

// actual implementation
Atlas::pointer inpaint(const Atlas &atlas, const Mesh &mesh
                       , int textureQuality)
{
    /** Construct hybrid atlas from any atlas.
     */
    opencv::Atlas in(atlas, textureQuality);

    auto out(std::make_shared<opencv::Atlas>(textureQuality));

    std::size_t index(0);
    for (const auto &sm : mesh) {
        auto tex(in.get(index++));

        cv::Mat mask(tex.rows, tex.cols, CV_8U, cv::Scalar(0x00));
        rasterizeMask(mask, sm.faces, denormalize(sm.tc, tex.size()));

        imgproc::jpegBlockInpaint(tex, mask);

        out->add(tex);
    }

    return out;
}

} } // namespace vtslibs::vts
