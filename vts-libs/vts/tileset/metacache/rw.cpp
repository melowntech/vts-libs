#include <map>

#include "dbglog/dbglog.hpp"

#include "../metacache.hpp"

namespace vtslibs { namespace vts {

namespace detail {

namespace {

typedef std::map<TileId, MetaTile::pointer> Map;

} // namespace

class RwMetaCache : public MetaCache {
public:
    RwMetaCache(const Driver::pointer &driver)
        : MetaCache(driver)
    {
        LOG(info1) << "RwMetaCache(" << driver->info() << ")";
    }

    virtual MetaTile::pointer add(const MetaTile::pointer &metatile) {
        auto res(map_.insert
                 (Map::value_type (metatile->origin(), metatile)));
        if (!res.second) {
            res.first->second = metatile;
        }
        return res.first->second;
    }

    virtual MetaTile::pointer find(const TileId &metaId) {
        auto fmap(map_.find(metaId));
        if (fmap != map_.end()) { return fmap->second; }
        return {};
    }

    virtual void clear() {
        map_.clear();
    }

    virtual void save() {
        for (const auto &item : map_) {
            const auto &meta(*item.second);
            auto tileId(meta.origin());
            LOG(info1) << "Saving: " << tileId;

            // save metatile to file
            auto f(driver_->output(tileId, TileFile::meta));
            meta.save(*f);
            f->close();
        }
    }

private:
    Map map_;
};

} // namespace detail

std::unique_ptr<MetaCache> MetaCache::rw(const Driver::pointer &driver)
{
    return std::unique_ptr<MetaCache>(new detail::RwMetaCache(driver));
}

} } // namespace vtslibs::vts
