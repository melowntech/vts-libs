#ifndef vtslibs_tilestorage_metatileop_hpp_included_
#define vtslibs_tilestorage_metatileop_hpp_included_

#include <boost/optional.hpp>

#include "./metatileop.hpp"
#include "./typesfwd.hpp"

namespace vtslibs { namespace vts0 {

void calcParams(MetaNode &metanode,const geometry::Obj &mesh
                , const math::Size2 &atlasSize
                , const boost::optional<double> &pixelSize = boost::none);

} } // namespace vtslibs::vts0

#endif // vtslibs_vts0_metatileop_hpp_included_
