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
    MetaCache(const Driver::pointer &driver) : driver_(driver) {}

    virtual ~MetaCache();

    virtual MetaTile::pointer add(const MetaTile::pointer &metatile) = 0;
    virtual MetaTile::pointer find(const TileId &metaId) = 0;
    virtual void clear() = 0;

    virtual void save() = 0;

    static std::unique_ptr<MetaCache> ro(const Driver::pointer &driver);
    static std::unique_ptr<MetaCache> rw(const Driver::pointer &driver);

    static std::unique_ptr<MetaCache> create(const Driver::pointer &driver);

protected:
    Driver::pointer driver_;
};

} } // namespace vadstena::vts

#endif // vadstena_libs_vts_tileset_metacache_hpp_included_
