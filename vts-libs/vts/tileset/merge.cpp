#include "./merge.hpp"

namespace vadstena { namespace vts { namespace merge {

const Mesh& Input::mesh() const
{
    if (!mesh_) {
        mesh_ = owner_->getMesh(tileId_, node_);
    }

    return *mesh_;
}

} } } // namespace vadstena::merge::vts
