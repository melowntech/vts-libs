#ifndef vadstena_libs_vts_navtile_hpp
#define vadstena_libs_vts_navtile_hpp

#include <cstdlib>
#include <memory>
#include <istream>

#include <boost/filesystem/path.hpp>

#include "math/geometry_core.hpp"

#include "../storage/range.hpp"

namespace vadstena { namespace vts {

class NavTile {
public:
    typedef std::shared_ptr<NavTile> pointer;
    typedef storage::Range<std::int16_t> HeightRange;

    static const math::Size2i size() { return math::Size2i(256, 256); };

    NavTile() {}

    virtual ~NavTile() {}

    virtual void serialize(std::ostream &os) const = 0;

    virtual void deserialize(const HeightRange &heightRange
                             , std::istream &is
                             , const boost::filesystem::path &path
                             = "unknown") = 0;

    HeightRange heightRange() const { return heightRange_; }

protected:
    HeightRange heightRange_;
};

} } // namespace vadstena::vts

#endif // vadstena_libs_vts_navtile_hpp
