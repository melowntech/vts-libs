#include "./atlas.hpp"

namespace vadstena { namespace vts {

void Atlas::serialize(std::ostream &out) const
{
    for (std::size_t i(0); i < count_; ++i) {
        serialize(out, i);
    }
}

void Atlas::deserialize(std::istream &in)
{
    for (std::size_t i(0); i < count_; ++i) {
        deserialize(in, i);
    }
}

} } // namespace vadstena::vts
