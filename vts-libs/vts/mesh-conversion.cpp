#include "./mesh-conversion.hpp"

namespace vadstena { namespace vts {

VerticesList convert(const Mesh &mesh, const geo::CsConvertor &conv)
{
    VerticesList out(mesh.submeshes.size());
    auto iout(out.begin());
    for (const auto &sm : mesh.submeshes) {
        auto &ov(*iout++);
        for (const auto &v : sm.vertices) {
            ov.push_back(conv(v));
        }
    }
    return out;
}

} } // namespace vadstena::vts
