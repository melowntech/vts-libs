#ifndef vtslibs_vts0_typesfwd_hpp_included_
#define vtslibs_vts0_typesfwd_hpp_included_

#include <opencv2/core/core.hpp>

#include "geometry/parse-obj.hpp"

#include "./basetypes.hpp"
#include "./metatile.hpp"

namespace vtslibs { namespace vts0 {

typedef geometry::Obj Mesh;
typedef cv::Mat Atlas;

} } // namespace vtslibs::vts0

#endif // vtslibs_vts0_typesfwd_hpp_included_
