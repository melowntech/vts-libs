#include <opencv2/highgui/highgui.hpp>

#include "imgproc/inpaint.hpp"

#include "../mesh.hpp"
#include "./atlas.hpp"
#include "./texture.hpp"

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
            (mask, sm.faces, opencv::denormalize(sm.tc, tex.size()));

        imgproc::jpegBlockInpaint(tex, mask);

        out->add(tex);
    }

    return out;
}

} } // namespace vtslibs::vts
