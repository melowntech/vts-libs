#ifndef vadstena_libs_vts_mesh_conversion_hpp
#define vadstena_libs_vts_mesh_conversion_hpp

#include "geo/csconvertor.hpp"

#include "./mesh.hpp"

namespace vadstena { namespace vts {

typedef std::vector<math::Points3d> VerticesList;

VerticesList convert(const Mesh &mesh, const geo::CsConvertor &conv);

} } // namespace vadstena::vts

#endif // vadstena_libs_vts_mesh_conversion_hpp
