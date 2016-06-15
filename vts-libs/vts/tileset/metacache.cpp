#include <boost/multi_index_container.hpp>
#include <boost/multi_index/ordered_index.hpp>
#include <boost/multi_index/identity.hpp>
#include <boost/multi_index/member.hpp>
#include <boost/multi_index/mem_fun.hpp>

#include "utility/time.hpp"

#include "./metacache.hpp"

namespace vadstena { namespace vts {

namespace limit {
std::size_t ReadOnlyMetatileLimit(1000);
std::atomic<std::size_t> roMetatiles(0);
} // namespace limit

namespace {

typedef decltype(utility::usecFromEpoch()) Time;
const Time MaxTime(std::numeric_limits<Time>::max());

struct Record {
    Record(const MetaTile::pointer &metatile)
        : metaId(metatile->origin()), lastHit(utility::usecFromEpoch())
        , metatile(metatile)
    {}

    void hit() { lastHit = utility::usecFromEpoch(); }

    TileId metaId;
    Time lastHit;
    MetaTile::pointer metatile;
};

struct MetaIdx {};
struct LastHitIdx {};

typedef boost::multi_index_container<
    Record
    , boost::multi_index::indexed_by<
          boost::multi_index::ordered_unique
          <boost::multi_index::tag<MetaIdx>
           , BOOST_MULTI_INDEX_MEMBER
           (Record, decltype(Record::metaId), metaId)>
          , boost::multi_index::ordered_non_unique
          <boost::multi_index::tag<LastHitIdx>
           , BOOST_MULTI_INDEX_MEMBER
           (Record, decltype(Record::lastHit), lastHit)>
          >
    > Map;

} // namespace

class MetaCache::Cache {
public:
    Cache(const Driver::pointer &driver)
        : driver_(driver), readOnly_(driver->readOnly())
    {}

    MetaTile::pointer add(const MetaTile::pointer &metatile) {
        Record record(metatile);
        auto res(map_.insert(record));
        if (!res.second) {
            map_.replace(res.first, record);
        } else if (readOnly_) {
            ++limit::roMetatiles;
        }

        // housekeeping, we want to keep found record
        houseKeeping(&record.metaId);
        return metatile;
    }

    MetaTile::pointer find(const TileId &metaId) {
        auto fmap(map_.find(metaId));
        if (fmap != map_.end()) {
            hit(map_, fmap);
            return fmap->metatile;
        }
        return {};
    }

    void clear() {
        limit::roMetatiles -= map_.size();
        map_.clear();
    }

    void save() {
        for (auto imap(map_.begin()), emap(map_.end()); imap != emap; ++imap) {
            const auto &meta(*imap->metatile);
            auto tileId(meta.origin());
            LOG(info1) << "Saving: " << tileId;

            // save metatile to file
            auto f(driver_->output(tileId, TileFile::meta));
            meta.save(*f);
            f->close();
        }
    }

private:
    void houseKeeping(const TileId *keep = nullptr);

    template <typename Idx, typename Iterator>
    inline void hit(Idx &idx, Iterator iterator) {
        idx.modify(iterator, [](Record &r) { r.hit(); });
    }

    Driver::pointer driver_;
    bool readOnly_;
    Map map_;
};

void MetaCache::Cache::houseKeeping(const TileId *keep)
{
    if (!readOnly_) {
        // rw case is unsupported so far
        return;
    }

    if (limit::roMetatiles <= limit::ReadOnlyMetatileLimit) {
        return;
    }

    auto &idx(map_.get<LastHitIdx>());

    int toDrop(5);
    for (auto iidx(idx.begin()); toDrop && (iidx != idx.end()); ) {
        // skip keep tile
        if (keep && (iidx->metaId == *keep)) {
            LOG(debug) << "Metatile " << iidx->metaId
                       << " must be kept in the cache.";
            ++iidx;
            continue;
        }

        // no changes -> we are free to remove the file
        LOG(debug) << "Removing metatile "
                   << iidx->metaId << '/' << iidx->lastHit
                   << " from the cache.";

        iidx = idx.erase(iidx);
        --toDrop;
        --limit::roMetatiles;
        break;
    }
}

MetaCache::MetaCache(const Driver::pointer &driver)
    : cache_(new Cache(driver))
{}

MetaCache::~MetaCache() {}

MetaTile::pointer MetaCache::add(const MetaTile::pointer &metatile)
{
    return cache_->add(metatile);
}

MetaTile::pointer MetaCache::find(const TileId &metaId)
{
    return cache_->find(metaId);
}

void MetaCache::clear()
{
    cache_->clear();
}

void MetaCache::save()
{
    cache_->save();
}

} } // namespace vadstena::vts
