#ifndef vadstena_libs_vts_mesh_conversion_hpp
#define vadstena_libs_vts_mesh_conversion_hpp

#include "geo/csconvertor.hpp"

#include "./mesh.hpp"

namespace vadstena { namespace vts {

typedef std::vector<math::Points3d> VerticesList;

/** Converts all submeshes' vertices by conv to another SRS and then by 4x4
 *  matrix to another system.
 */
VerticesList convert(const Mesh &mesh, const geo::CsConvertor &conv
                     , const math::Matrix4 &trafo
                     = boost::numeric::ublas::identity_matrix<double>(4));

} } // namespace vadstena::vts

#endif // vadstena_libs_vts_mesh_conversion_hpp
