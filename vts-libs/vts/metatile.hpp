#ifndef vadstena_libs_vts_metatile_hpp
#define vadstena_libs_vts_metatile_hpp

#include <cstdint>
#include <iosfwd>
#include <vector>

#include <boost/filesystem/path.hpp>

#include "math/geometry_core.hpp"
#include "half/half.hpp"

#include "../storage/error.hpp"

#include "./basetypes.hpp"

#include "../range.hpp"

typedef half_float::half hfloat;

namespace vadstena { namespace vts {

struct MetaNode {
    bool hasGeometry;
    bool hasNavtile;
    bool hasInternalTexture;
    enum class CoarsenessControl { displaySize, texelSize };
    CoarsenessControl cc;

    union {
        std::uint16_t displaySize;
        hfloat meshArea;
    };

    hfloat textureArea;

    Range<std::uint16_t> heightRange;

    std::vector<std::uint16_t> credits;

    MetaNode()
        : hasGeometry(), hasNavtile(), hasInternalTexture()
        , cc(CoarsenessControl::displaySize)
        , textureArea(), heightRange()
    {}
};

class MetaTile {
public:
    MetaTile(const TileId &origin, std::uint8_t binaryOrder)
        : origin_(origin), binaryOrder_(binaryOrder)
        , size_(1 << binaryOrder)
    {}

    MetaNode& get(const TileId &tileId);

    const MetaNode& get(const TileId &tileId) const;

private:
    std::size_t index(const TileId &tileId) const;

    /** Metatile ID, origin of first node in the metatile.
     */
    TileId origin_;

    std::uint8_t binaryOrder_;

    std::size_t size_;

    /** Contains (1 << binaryOrder_) * (1 << binaryOrder_) meta nodes
     */
    std::vector<MetaNode> grid_;
};

// inlines

MetaNode& MetaTile::get(const TileId &tileId)
{
    return grid_[index(tileId)];
}

const MetaNode& MetaTile::get(const TileId &tileId) const
{
    return grid_[index(tileId)];
}

} } // namespace vadstena::vts

#endif // vadstena_libs_vts_metatile_hpp
