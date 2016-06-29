#ifndef vadstena_libs_vts_2d_hpp_included_
#define vadstena_libs_vts_2d_hpp_included_

#include <boost/gil/gil_all.hpp>

#include "./types2d.hpp"
#include "./mesh.hpp"

namespace vadstena { namespace vts {

typedef boost::gil::gray8_image_t MaskImage;

MaskImage mask2d(const MeshMask &mask);

} } // vadstena::vts

#endif // vadstena_libs_vts_2d_hpp_included_
