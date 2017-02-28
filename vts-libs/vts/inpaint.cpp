#include "dbglog/dbglog.hpp"

#include "../storage/error.hpp"

#include "./atlas.hpp"

namespace vtslibs { namespace vts {

Atlas::pointer inpaint(const Atlas &atlas, const Mesh &mesh
                       , int textureQuality)
{
    (void) atlas;
    (void) mesh;
    (void) textureQuality;

    LOGTHROW(warn2, storage::Unimplemented)
        << "Atlas inpaint is not available without OpenCV support.";

    return {};
}

} } // namespace vtslibs::vts
