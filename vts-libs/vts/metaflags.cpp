#include "./metaflags.hpp"

namespace vtslibs { namespace vts {

std::vector<MetaFlags::MetaFlag> MetaFlags::mapping = {
    MetaFlag(vts::MetaNode::Flag::geometryPresent, "geometry")
    , MetaFlag(vts::MetaNode::Flag::alien, "alien")
    , MetaFlag(vts::MetaNode::Flag::navtilePresent, "navtile")
    , MetaFlag(vts::MetaNode::Flag::applyTexelSize, "texelSize")
    , MetaFlag(vts::MetaNode::Flag::applyDisplaySize, "displaySize")
    , MetaFlag(vts::MetaNode::Flag::ulChild, "ul")
    , MetaFlag(vts::MetaNode::Flag::urChild, "ur")
    , MetaFlag(vts::MetaNode::Flag::llChild, "ll")
    , MetaFlag(vts::MetaNode::Flag::lrChild, "lr")
};

} } // namespace vtslibs::vts
