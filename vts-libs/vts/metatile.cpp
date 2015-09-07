#include "dbglog/dbglog.hpp"

#include "./metatile.hpp"
#include "./io.hpp"

namespace vadstena { namespace vts {

std::size_t MetaTile::index(const TileId &tileId) const
{
    if ((origin_.lod != tileId.lod)
        || (origin_.x > tileId.x)
        || (origin_.y > tileId.y))
    {
        LOGTHROW(err1, storage::NoSuchTile)
            << "Node " << tileId << " not inside metatile " << origin_
            << ".";
    }

    std::size_t x(tileId.x - origin_.x);
    std::size_t y(tileId.y - origin_.y);
    if ((x >= size_) || (y >= size_)) {
        LOGTHROW(err1, storage::NoSuchTile)
            << "Node " << tileId << " not inside metatile " << origin_
            << ".";
    }

    return y * size_ + x;
}

} } // namespace vadstena::vts

