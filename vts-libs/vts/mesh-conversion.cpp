#include "dbglog/dbglog.hpp"

#include "math/transform.hpp"

#include "./mesh-conversion.hpp"

namespace vadstena { namespace vts {

VerticesList convert(const Mesh &mesh, const geo::CsConvertor &conv
                     , const math::Matrix4 &trafo)
{
    VerticesList out(mesh.submeshes.size());
    auto iout(out.begin());
    for (const auto &sm : mesh.submeshes) {
        auto &ov(*iout++);
        for (const auto &v : sm.vertices) {
            ov.push_back(transform(trafo, conv(v)));
        }
    }
    return out;
}

} } // namespace vadstena::vts
