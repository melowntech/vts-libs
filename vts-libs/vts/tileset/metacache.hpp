/**
 * \file vts/metacache.hpp
 * \author Vaclav Blazek <vaclav.blazek@citationtech.net>
 *
 * Metatile cache.
 */

#ifndef vadstena_libs_vts_tileset_metacache_hpp_included_
#define vadstena_libs_vts_tileset_metacache_hpp_included_

#include "../metatile.hpp"
#include "./driver.hpp"

namespace vadstena { namespace vts {

class MetaCache {
public:
    MetaCache(const Driver::pointer &driver);
    ~MetaCache();

    MetaTile::pointer add(const MetaTile::pointer &metatile);
    MetaTile::pointer find(const TileId &metaId);
    void clear();

    void save();

private:
    struct Cache;
    std::unique_ptr<Cache> cache_;
};

} } // namespace vadstena::vts

#endif // vadstena_libs_vts_tileset_metacache_hpp_included_
