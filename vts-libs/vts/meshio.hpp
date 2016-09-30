#ifndef vadstena_libs_vts_meshio_hpp
#define vadstena_libs_vts_meshio_hpp

#include "mesh.hpp"

namespace vadstena { namespace vts { namespace detail {


void saveMeshProper(std::ostream &out, const Mesh &mesh);

void loadMeshProper(std::istream &in, const boost::filesystem::path &path
                    , Mesh &mesh);


} } } // namespace vadstena::vts::detail

#endif // vadstena_libs_vts_meshio_hpp
