#ifndef vadstena_libs_tilestorage_typesfwd_hpp_included_
#define vadstena_libs_tilestorage_typesfwd_hpp_included_

#include <opencv2/core/core.hpp>

#include "geometry/parse-obj.hpp"

#include "./basetypes.hpp"
#include "./metatile.hpp"

namespace vadstena { namespace tilestorage {

typedef geometry::Obj Mesh;
typedef cv::Mat Atlas;

} } // namespace vadstena::tilestorage

#endif // vadstena_libs_tilestorage_typesfwd_hpp_included_
