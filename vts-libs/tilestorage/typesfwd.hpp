#ifndef vtslibs_tilestorage_typesfwd_hpp_included_
#define vtslibs_tilestorage_typesfwd_hpp_included_

#include <opencv2/core/core.hpp>

#include "geometry/parse-obj.hpp"

#include "./basetypes.hpp"
#include "./metatile.hpp"

namespace vtslibs { namespace tilestorage {

typedef geometry::Obj Mesh;
typedef cv::Mat Atlas;

} } // namespace vtslibs::tilestorage

#endif // vtslibs_tilestorage_typesfwd_hpp_included_
