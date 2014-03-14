#include "./traverser.hpp"
#include "./tileindex.hpp"

namespace vadstena { namespace tilestorage {

Traverser::Traverser(const TileIndex *owner)
    : owner_(owner), mask_(), index_(owner_->minLod())
{
    load();
}

void Traverser::load()
{
    mask_ = owner_->mask(index_.lod);
    if (!mask_) { return; }
    auto size(mask_->dims());
    size_ = Size2l(size.width, size.height);
    index_.easting = index_.northing = 0;
}

Traverser::Tile Traverser::next()
{
    if (!mask_ ) { return {}; }
    Tile t(index_, owner_->tileId(index_)
           , mask_->get(index_.easting, index_.northing));

    // increment
    ++index_.easting;
    if (index_.easting >= size_.width) {
        index_.easting = 0;
        ++index_.northing;
    }

    if (index_.northing >= size_.height) {
        ++index_.lod;
        load();
    }

    return t;
}

} } // namespace vadstena::tilestorage
