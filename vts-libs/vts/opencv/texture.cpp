/**
 * Copyright (c) 2017 Melown Technologies SE
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * *  Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#include <opencv2/imgproc/imgproc.hpp>

#include "imgproc/scanconversion.hpp"
#include "imgproc/fillrect.hpp"

#include "texture.hpp"

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

    auto paint([&](int x, int y, int w, int h) -> void
    {
        imgproc::fillRectangle(mask, cv::Rect(x, y, w, h), white);
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
