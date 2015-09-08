#include "./tileset.hpp"
#include "./tileset/detail.hpp"
#include "./tileset/driver.hpp"

namespace vadstena { namespace vts {

TileSet::TileSet(const std::shared_ptr<Driver> &driver)
{
    (void) driver;
}

TileSet::~TileSet()
{
}

Mesh TileSet::mesh(const TileId &tileId) const
{
    (void) tileId;
    return {};
}

void TileSet::mesh(const TileId &tileId, const Mesh &mesh) const
{
    (void) tileId;
    (void) mesh;
}

Atlas TileSet::atlas(const TileId &tileId) const
{
    (void) tileId;
    return {};
}

void TileSet::atlas(const TileId &tileId, const Atlas &atlas) const
{
    (void) tileId;
    (void) atlas;
}

MetaNode TileSet::metaNode(const TileId &tileId) const
{
    (void) tileId;
    return {};
}

void TileSet::metaNode(const TileId &tileId, const MetaNode node) const
{
    (void) tileId;
    (void) node;
}

bool TileSet::exists(const TileId &tileId) const
{
    (void) tileId;
    return false;
}

void TileSet::flush()
{
}

void TileSet::begin(utility::Runnable *runnable)
{
    (void) runnable;
}

void TileSet::commit()
{
}

void TileSet::rollback()
{
}

void TileSet::watch(utility::Runnable *runnable)
{
    (void) runnable;
}

bool TileSet::inTx() const
{
    return false;
}

bool TileSet::empty() const
{
    return true;
}

void TileSet::drop()
{
}

LodRange TileSet::lodRange() const
{
    return {};
}

} } // namespace vadstena::vts
