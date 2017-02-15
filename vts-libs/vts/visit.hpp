#ifndef vtslibs_vts_visit_hpp_included_
#define vtslibs_vts_visit_hpp_included_

#include <memory>

#include <boost/optional.hpp>
#include <boost/noncopyable.hpp>

#include "dbglog/dbglog.hpp"

#include "./tileset.hpp"
#include "./nodeinfo.hpp"

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
