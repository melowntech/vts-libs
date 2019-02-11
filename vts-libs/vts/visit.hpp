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
#ifndef vtslibs_vts_visit_hpp_included_
#define vtslibs_vts_visit_hpp_included_

#include <memory>

#include <boost/optional.hpp>
#include <boost/noncopyable.hpp>

#include "dbglog/dbglog.hpp"

#include "tileset.hpp"
#include "nodeinfo.hpp"

namespace vtslibs { namespace vts {

/** Recursively visits all tileset's nodes from the root.
 *
 * Visitor signature:
 * bool visitor(TileIndex::Flag::value_type flags, const NodeInfo &nodeInfo)
 *
 * Descend is stopped when hitting invalid/nonexistent node or visitor return
 * false.
 *
 * \param tileset to visit
 * \param visitor visitor to call on each valid node
 */
template <typename Visitor>
void visit(const TileSet &tileset, const Visitor &visitor);

// implementation

namespace detail {

/** Visits given node.
 *
 * \param fti full tileindex, world to traverse
 * \param ti tileindex, used to get tile flags from
 * \param ni current node's info
 * \param visitor visitor to call
 */
template <typename Visitor>
void visitDescend(const TileIndex &fti, const TileIndex &ti
                  , const NodeInfo &ni, const Visitor &visitor)
{
    const auto &tileId(ni.nodeId());

    if (!ni.valid()) { return; }

    if (!fti.get(tileId)) { return; }

    if (!visitor(ti.get(tileId), ni)) { return; }

    for (auto child : children(tileId)) {
        visitDescend(fti, ti, ni.child(child), visitor);
    }
}

} // namespace detail

template <typename Visitor>
void visit(const TileSet &tileset, const Visitor &visitor)
{
    const auto &ti(tileset.tileIndex());
    auto fti(ti);
    fti.makeAbsolute().complete();
    detail::visitDescend(fti, ti, tileset.rootNode(), visitor);
}

} } // namespace vtslibs::vts

#endif // vtslibs_vts_visit_hpp_included_
