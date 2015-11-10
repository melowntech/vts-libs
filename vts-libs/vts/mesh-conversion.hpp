#ifndef vadstena_libs_vts_mesh_conversion_hpp
#define vadstena_libs_vts_mesh_conversion_hpp

#include "geo/csconvertor.hpp"

#include "./mesh.hpp"

namespace vadstena { namespace vts {

typedef std::vector<math::Points2d> Vertices2List;
typedef std::vector<math::Points3d> Vertices3List;

/** Converts all submeshes' vertices by conv to another SRS and then by 4x4
 *  matrix to another system. Result is in 2d points.
 *
 * Both convertor and matrix can be NULL -> no operation.
 */
Vertices2List convert2(const Mesh &mesh, const geo::CsConvertor *conv
                       , const math::Matrix4 *trafo);

/** Converts all submeshes' vertices by conv to another SRS and then by 4x4
 *  matrix to another system.
 *
 * Both convertor and matrix can be NULL -> no operation.
 */
Vertices3List convert3(const Mesh &mesh, const geo::CsConvertor *conv
                       , const math::Matrix4 *trafo);

} } // namespace vadstena::vts

#endif // vadstena_libs_vts_mesh_conversion_hpp
