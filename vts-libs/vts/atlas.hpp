#ifndef vadstena_libs_vts_atlas_hpp
#define vadstena_libs_vts_atlas_hpp

#include <cstdlib>
#include <iostream>

namespace vadstena { namespace vts {

class Atlas {
public:
    Atlas(std::size_t count) : count_(count) {}

    virtual ~Atlas() {}

    std::size_t count() const { return count_; }

    virtual void serialize(std::ostream &out, std::size_t index) const = 0;

    virtual void deserialize(std::istream &in, std::size_t index) = 0;

    void serialize(std::ostream &out) const;

    void deserialize(std::istream &in);

protected:
    std::size_t count_;
};

} } // namespace vadstena::vts

#endif // vadstena_libs_vts_atlas_hpp
