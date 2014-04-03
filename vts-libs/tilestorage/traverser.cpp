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
    // load next non-empty mask
    for (;;) {
        mask_ = owner_->mask(index_.lod);
        if (!mask_) { return; }
        if (!mask_->empty()) { return; }
        ++index_.lod;
    }

    auto size(mask_->dims());
    size_ = Size2l(size.width, size.height);
    index_.easting = index_.northing = 0;
}

void Traverser::increment()
{
    ++index_.easting;
    if (index_.easting >= size_.width) {
        index_.easting = 0;
        ++index_.northing;
    }

    if (index_.northing >= size_.height) {
        ++index_.lod;
        load();
    }
}

Traverser::Tile Traverser::next()
{
    for (;;) {
        if (!mask_ ) { return {}; }
        bool value(mask_->get(index_.easting, index_.northing));
        Tile t(index_, owner_->tileId(index_));

        increment();
        if (value) { return t; }
    }
}

} } // namespace vadstena::tilestorage
