/**
 * Copyright (c) 2017 Melown Technologies SE
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * *  Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
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
