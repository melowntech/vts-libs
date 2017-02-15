#ifndef vtslibs_vts_meshio_hpp
#define vtslibs_vts_meshio_hpp

#include "mesh.hpp"

namespace vtslibs { namespace vts { namespace detail {


void saveMeshProper(std::ostream &out, const Mesh &mesh
                    , const Atlas *atlas);

void loadMeshProper(std::istream &in, const boost::filesystem::path &path
                    , Mesh &mesh);


} } } // namespace vtslibs::vts::detail

#endif // vtslibs_vts_meshio_hpp
