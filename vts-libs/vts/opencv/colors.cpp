#include "./colors.hpp"

namespace vtslibs { namespace vts { namespace opencv {

const cv::Scalar palette256[256] = {
    #include "../colors.incl.cpp"
};

} } } // namespace vtslibs::vts::opencv
