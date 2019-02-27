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
#include <opencv2/imgproc/imgproc.hpp>

#include "dbglog/dbglog.hpp"
#include "utility/binaryio.hpp"

#include "imgproc/fillrect.hpp"

#include "qtree.hpp"

namespace fs = boost::filesystem;
namespace bin = utility::binaryio;

namespace vtslibs { namespace vts {

void dump(const QTree &tree, const fs::path &path
          , const std::function<bool(QTree::value_type)> &filter
          , double pixelSize)
{
    const auto size(tree.size());
    cv::Mat m(long(std::ceil(pixelSize * size.height))
              , long(std::ceil(pixelSize * size.width))
              , CV_8UC1);
    m = cv::Scalar(0);
    auto white(cv::Scalar(0xff));

    uint fracAdj(std::round(pixelSize) - pixelSize != 0.0 ? 1 : 0);

    tree.forEachNode([&](unsigned int x, unsigned int y, unsigned int size
                         , QTree::value_type value)
    {
        if (!(filter(value))) { return; }

        cv::Point2i start(int(std::floor(pixelSize * x))
                          , int(std::floor(pixelSize * y)));
        cv::Point2i end
            (int(std::ceil(pixelSize * (x + size - fracAdj )))
             , int(std::ceil(pixelSize * (y + size - fracAdj))));

        imgproc::fillRectangle(m, start, end, white);
    }, QTree::Filter::white);

    imwrite(path.string(), m);
}

} } // namespace vtslibs::vts
