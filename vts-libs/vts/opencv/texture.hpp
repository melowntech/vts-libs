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

void dilate(cv::Mat &mask, int distance);

} } } // namespace vtslibs::vts::opencv

#endif // vtslibs_vts_opencv_texture_hpp
