#include "./colors.hpp"

namespace vadstena { namespace vts { namespace opencv {

const cv::Scalar palette256[256] = {
    #include "../colors.incl.cpp"
};

} } } // namespace vadstena::vts::opencv
