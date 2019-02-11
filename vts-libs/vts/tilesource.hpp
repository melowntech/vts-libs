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
#ifndef vtslibs_vts_tilesource_hpp_included_
#define vtslibs_vts_tilesource_hpp_included_

#include "../storage/streams.hpp"

#include "metatile.hpp"
#include "tileindex.hpp"

namespace vtslibs { namespace vts {

using storage::IStream;

/** Tile source. Can be used to pump tile between tilesets.
 */
struct TileSource {
    MetaNode metanode;
    TileIndex::Flag::value_type extraFlags;

    IStream::pointer mesh;
    IStream::pointer atlas;
    IStream::pointer navtile;

    TileSource() : extraFlags(0) {}
    TileSource(const MetaNode &metanode
               , TileIndex::Flag::value_type extraFlags)
        : metanode(metanode), extraFlags(extraFlags)
    {}
};

} } // namespace vtslibs::vts

#endif // vtslibs_vts_tilesource_hpp_included_
