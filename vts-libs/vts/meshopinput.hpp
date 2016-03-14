/**
 * \file vts/meshopinput.hpp
 * \author Vaclav Blazek <vaclav.blazek@citationtech.net>
 *
 * Mesh operation input.
 */

#ifndef vadstena_libs_vts_meshop_hpp_included_
#define vadstena_libs_vts_meshop_hpp_included_

#include <boost/optional.hpp>

#include "./basetypes.hpp"
#include "./tileop.hpp"
#include "./atlas.hpp"
#include "./opencv/navtile.hpp"
#include "./tileset.hpp"

namespace vadstena { namespace vts {

/** Mesh operation input.
 */
class MeshOpInput {
public:
    typedef int Id;

    /** Create meshop input.
     *
     *  \param id owner identifier; used for sorting purposes
     *  \param owner owner of the tile
     *  \param tileId tile identifier
     *  \param nodeInfo node info (fetched from tileset if null)
     *  \param lazy loads data on demand
     */
    MeshOpInput(Id id, const TileSet::Detail &owner, const TileId &tileId
                , const NodeInfo *nodeInfo = nullptr, bool lazy = true);

    /** Create meshop input.
     *
     *  \param id owner identifier; used for sorting purposes
     *  \param owner owner of the tile
     *  \param tileId tile identifier
     *  \param nodeInfo node info (fetched from tileset if null)
     *  \param lazy loads data on demand
     */
    MeshOpInput(Id id, const TileSet &owner, const TileId &tileId
                , const NodeInfo *nodeInfo = nullptr, bool lazy = true);

    /** Input is valid only if there is node with geometry
     */
    operator bool() const { return hasMesh(); }

    const MetaNode& node() const;
    const NodeInfo& nodeInfo() const { return *nodeInfo_; }

    bool watertight() const;
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

    typedef std::vector<MeshOpInput> list;

    const math::Matrix4 sd2Coverage(const NodeInfo &nodeInfo) const;

    const math::Matrix4 coverage2Sd(const NodeInfo &nodeInfo) const;

    const math::Matrix4 coverage2Texture() const;

    /** Return owning tileset
     */
    const TileSet::Detail *owner() const { return owner_; }

    const std::string& name() const;

    const TileId& tileId() const { return tileId_; }

    Id id() const { return id_; }

    bool operator<(const MeshOpInput &o) const { return id_ < o.id_; }

private:
    void prepare(bool lazy);

    bool loadNode() const;

    Id id_;

    /** Tile's ID
     */
    TileId tileId_;

    /** Difference between this tileId and tileId of currently processed tile.
     *  Default to (0, 0, 0).
     */
    TileId tileDiff_;
    const TileSet::Detail *owner_;
    TileIndex::Flag::value_type flags_;
    const NodeInfo *nodeInfo_;

    mutable bool nodeLoaded_;
    mutable const MetaNode *node_;
    mutable boost::optional<Mesh> mesh_;
    mutable boost::optional<RawAtlas> atlas_;
    mutable boost::optional<opencv::NavTile> navtile_;

    /** Valid only when not using exernal node info
     */
    boost::optional<NodeInfo> ownNodeInfo_;
};

} } // namespace vadstena::vts

#endif // vadstena_libs_vts_meshop_hpp_included_
