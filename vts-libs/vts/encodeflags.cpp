#include "./encodeflags.hpp"

namespace vtslibs { namespace vts {

std::vector<EncodeFlags::EncodeFlag> EncodeFlags::mapping = {
    EncodeFlag(vts::CloneOptions::EncodeFlag::mesh, "mesh")
    , EncodeFlag(vts::CloneOptions::EncodeFlag::inpaint, "inpaint")
    , EncodeFlag(vts::CloneOptions::EncodeFlag::meta, "meta")
};

} } // namespace vtslibs::vts
