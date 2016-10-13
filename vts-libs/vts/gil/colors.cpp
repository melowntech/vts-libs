#include "./colors.hpp"

namespace vadstena { namespace vts { namespace gil {

const boost::gil::rgb8_pixel_t palette256[256] = {
    #include "../colors.incl.cpp"
};

} } } // namespace vadstena::vts::gil
