#ifndef vtslibs_tilestorage_metatileop_hpp_included_
#define vtslibs_tilestorage_metatileop_hpp_included_

#include <boost/optional.hpp>

#include "./metatileop.hpp"
#include "./typesfwd.hpp"

namespace vtslibs { namespace tilestorage {

void calcParams(MetaNode &metanode,const geometry::Obj &mesh
                , const math::Size2 &atlasSize
                , const boost::optional<double> &pixelSize = boost::none);

} } // namespace vtslibs::tilestorage

#endif // vtslibs_tilestorage_metatileop_hpp_included_
