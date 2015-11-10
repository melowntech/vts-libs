#include "dbglog/dbglog.hpp"

#include "math/transform.hpp"

#include "./mesh-conversion.hpp"

namespace vadstena { namespace vts {

Vertices2List convert2(const Mesh &mesh, const geo::CsConvertor *conv
                       , const math::Matrix4 *trafo)
{
    Vertices2List out(mesh.submeshes.size());
    auto iout(out.begin());
    for (const auto &sm : mesh.submeshes) {
        auto &ov(*iout++);
        for (const auto &v : sm.vertices) {
            // apply conversion (if defined)
            if (conv) {
                if (trafo) {
                    ov.push_back(transform(*trafo, (*conv)(v)));
                } else {
                    ov.push_back((*conv)(v));
                }
            } else {
                if (trafo) {
                    ov.push_back(transform(*trafo, v));
                } else {
                    ov.push_back(v);
                }
            }
        }
    }
    return out;
}

Vertices3List convert3(const Mesh &mesh, const geo::CsConvertor *conv
                       , const math::Matrix4 *trafo)
{
    Vertices3List out(mesh.submeshes.size());
    auto iout(out.begin());
    for (const auto &sm : mesh.submeshes) {
        auto &ov(*iout++);
        for (const auto &v : sm.vertices) {
            // apply conversion (if defined)
            if (conv) {
                if (trafo) {
                    ov.push_back(transform(*trafo, (*conv)(v)));
                } else {
                    ov.push_back((*conv)(v));
                }
            } else {
                if (trafo) {
                    ov.push_back(transform(*trafo, v));
                } else {
                    ov.push_back(v);
                }
            }
        }
    }
    return out;
}

} } // namespace vadstena::vts
