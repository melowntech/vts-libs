#ifndef vadstena_libs_vts0_typesfwd_hpp_included_
#define vadstena_libs_vts0_typesfwd_hpp_included_

#include <opencv2/core/core.hpp>

#include "geometry/parse-obj.hpp"

#include "./basetypes.hpp"
#include "./metatile.hpp"

namespace vadstena { namespace vts0 {

typedef geometry::Obj Mesh;
typedef cv::Mat Atlas;

} } // namespace vadstena::vts0

#endif // vadstena_libs_vts0_typesfwd_hpp_included_
