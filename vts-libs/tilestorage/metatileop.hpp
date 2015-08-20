#ifndef vadstena_libs_tilestorage_metatileop_hpp_included_
#define vadstena_libs_tilestorage_metatileop_hpp_included_

#include <boost/optional.hpp>

#include "./metatileop.hpp"
#include "./typesfwd.hpp"

namespace vadstena { namespace tilestorage {

void calcParams(MetaNode &metanode,const geometry::Obj &mesh
                , const math::Size2 &atlasSize
                , const boost::optional<double> &pixelSize = boost::none);

} } // namespace vadstena::tilestorage

#endif // vadstena_libs_tilestorage_metatileop_hpp_included_
