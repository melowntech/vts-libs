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
#include "../opencv/atlas.hpp"
#include "../opencv/navtile.hpp"
#include "../mesh-conversion.hpp"
#include "./detail.hpp"

namespace vadstena { namespace vts { namespace merge {

/** Merge input.
 */
class Input {
public:
    typedef int Id;

    /** Create merge input.
     *
     *  \param id owner identifier; used for sorting purposes
     *  \param owner owner of the tile
     *  \param tileId tile identifier
     */
    Input(Id id, const TileSet::Detail &owner, const TileId &tileId
          , const NodeInfo &nodeInfo)
        : id_(id), tileId_(tileId), owner_(&owner)
        , node_(owner.findMetaNode(tileId))
        , nodeInfo_(&nodeInfo)
    {}

    /** Input is valid only if there is node with geometry
     */
    operator bool() const { return node_ && node_->geometry(); }

    /** Must not be called when operator bool() returns false!
     */
    const MetaNode& node() const { return *node_; }

    bool hasMesh() const;
    bool hasAtlas() const;
    bool hasNavtile() const;

    /** Returns mesh. Lazy load.
     */
    const Mesh& mesh() const;

    /** Returns atlas. Lazy load.
     */
    const RawAtlas& atlas() const;

    /** Returns atlas. Lazy load.
     */
    const opencv::NavTile& navtile() const;

    typedef std::vector<Input> list;

    /** Returns mesh vertices (vector per submesh) converted to spatial division
     *  SRS.
     */
    const VerticesList& sdVertices() const;

    /** Return owning tileset
     */
    const TileSet::Detail *owner() const { return owner_; }

    const TileId& tileId() const { return tileId_; }

    Id id() const { return id_; }

    bool operator<(const Input &o) const { return id_ < o.id_; }

private:
    Id id_;
    TileId tileId_;
    const TileSet::Detail *owner_;

    const MetaNode *node_;
    const NodeInfo *nodeInfo_;
    mutable boost::optional<Mesh> mesh_;
    mutable boost::optional<RawAtlas> atlas_;
    mutable boost::optional<opencv::NavTile> navtile_;

    /** Mesh vertices in spatial division SRS
     */
    mutable boost::optional<VerticesList> sdVertices_;
};

/** Merge output.
 */
struct Output {
    MetaNode node;
    boost::optional<Mesh> mesh;
    boost::optional<RawAtlas> atlas;
    boost::optional<opencv::NavTile> navtile;

    // list of tiles this tile was generated from
    Input::list source;

    operator bool() const {
        return mesh || atlas || navtile;
    }

    const Mesh* getMesh() const { return mesh ? &*mesh : nullptr; }
    const Atlas* getAtlas() const { return atlas ? &*atlas : nullptr; }
    const NavTile* getNavtile() const { return navtile ? &*navtile : nullptr; }
};

/** Generates new tile from given source and parent source fallback.
 *
 * Source and * parent source inputs are merged together using their id's.
 */
Output mergeTile(const Input::list &source
                 , const Input::list &parentSource, int quadrant);

} } } // namespace vadstena::merge::vts

#endif // vadstena_libs_vts_merge_hpp_included_
