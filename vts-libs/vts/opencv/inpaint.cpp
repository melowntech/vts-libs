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
#include <opencv2/highgui/highgui.hpp>

#include "imgproc/inpaint.hpp"

#include "../mesh.hpp"
#include "atlas.hpp"
#include "texture.hpp"

namespace vtslibs { namespace vts {

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
        opencv::rasterizeMask
            (mask, sm.facesTc, opencv::denormalize(sm.tc, tex.size()));
        opencv::dilate(mask, 1);

        imgproc::jpegBlockInpaint(tex, mask);

        out->add(tex);
    }

    return out;
}

} } // namespace vtslibs::vts
