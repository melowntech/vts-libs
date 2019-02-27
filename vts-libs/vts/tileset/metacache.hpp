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
/**
 * \file vts/metacache.hpp
 * \author Vaclav Blazek <vaclav.blazek@citationtech.net>
 *
 * Metatile cache.
 */

#ifndef vtslibs_vts_tileset_metacache_hpp_included_
#define vtslibs_vts_tileset_metacache_hpp_included_

#include "../metatile.hpp"
#include "driver.hpp"

namespace vtslibs { namespace vts {

class MetaCache {
public:
    virtual ~MetaCache();

    virtual MetaTile::pointer add(const MetaTile::pointer &metatile) = 0;
    virtual MetaTile::pointer find(const TileId &metaId) = 0;
    virtual void clear() = 0;

    virtual void save() = 0;

    static std::unique_ptr<MetaCache> create(const Driver::pointer &driver);

protected:
    MetaCache(const Driver::pointer &driver) : driver_(driver) {}

    static std::unique_ptr<MetaCache> ro(const Driver::pointer &driver);
    static std::unique_ptr<MetaCache> rw(const Driver::pointer &driver);
    static std::unique_ptr<MetaCache>
    roScarceMemory(const Driver::pointer &driver);

    Driver::pointer driver_;
};

} } // namespace vtslibs::vts

#endif // vtslibs_vts_tileset_metacache_hpp_included_
