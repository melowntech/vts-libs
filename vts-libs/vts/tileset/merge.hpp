/**
 * \file vts/merge.hpp
 * \author Vaclav Blazek <vaclav.blazek@citationtech.net>
 *
 * Tile content merge.
 */

#ifndef vadstena_libs_vts_merge_hpp_included_
#define vadstena_libs_vts_merge_hpp_included_

#include <boost/optional.hpp>

#include "../basetypes.hpp"
#include "../tileop.hpp"
#include "./detail.hpp"

namespace vadstena { namespace vts { namespace merge {

struct Input {
    Input(const TileSet::Detail &owner, const TileId &tileId)
        : tileId_(tileId), owner_(&owner)
        , node_(owner.findMetaNode(tileId))
    {}

    /** Input is valid only if there is node with geometry
     */
    operator bool() const { return node_ && node_->geometry(); }

    /** Must not be called when operator bool() returns false!
     */
    const MetaNode& node() const { return *node_; }

    /** Returns mesh. Lazy load.
     */
    const Mesh& mesh() const;

    typedef std::vector<Input> list;

private:
    TileId tileId_;
    const TileSet::Detail *owner_;

    const MetaNode *node_;
    mutable boost::optional<Mesh> mesh_;
};

} } } // namespace vadstena::merge::vts

#endif // vadstena_libs_vts_merge_hpp_included_
