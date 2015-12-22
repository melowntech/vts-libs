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
#include "./detail.hpp"

namespace vadstena { namespace vts { namespace merge {

typedef std::vector<math::Points3d> Vertices3List;

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
          , const NodeInfo &nodeInfo);

    /** Input is valid only if there is node with geometry
     */
    operator bool() const { return node_ && node_->geometry(); }

    /** Must not be called when operator bool() returns false!
     */
    const MetaNode& node() const { return *node_; }

    const NodeInfo& nodeInfo() const { return *nodeInfo_; }

    bool hasMesh() const;
    bool hasAtlas() const;
    bool hasNavtile() const;

    /** Returns mesh. Lazy load.
     */
    const Mesh& mesh() const;

    /** Returns atlas. Lazy load.
     */
    const opencv::RawAtlas& atlas() const;

    /** Returns atlas. Lazy load.
     */
    const opencv::NavTile& navtile() const;

    typedef std::vector<Input> list;

    const math::Matrix4 sd2Coverage(const NodeInfo &nodeInfo) const;

    const math::Matrix4 coverage2Sd(const NodeInfo &nodeInfo) const;

    const math::Matrix4 coverage2Texture() const;

    /** Return owning tileset
     */
    const TileSet::Detail *owner() const { return owner_; }

    const std::string& name() const { return owner_->properties.id; }

    const TileId& tileId() const { return tileId_; }

    Id id() const { return id_; }

    bool operator<(const Input &o) const { return id_ < o.id_; }

private:
    Id id_;

    /** Tile's ID
     */
    TileId tileId_;

    /** Difference between this tileId and tileId of currently processed tile.
     *  Default to (0, 0, 0).
     */
    TileId tileDiff_;
    const TileSet::Detail *owner_;

    const MetaNode *node_;
    const NodeInfo *nodeInfo_;
    mutable boost::optional<Mesh> mesh_;
    mutable boost::optional<opencv::RawAtlas> atlas_;
    mutable boost::optional<opencv::NavTile> navtile_;
};

/** Merge output.
 */
struct Output {
    TileId tileId;

    MetaNode node;
    boost::optional<Mesh> mesh;
    boost::optional<opencv::RawAtlas> atlas;
    boost::optional<opencv::NavTile> navtile;

    // list of tiles this tile was generated from
    Input::list source;

    explicit Output(const TileId &tileId) : tileId(tileId) {}

    Output(const TileId &tileId, const Input &input)
        : tileId(tileId), source{input} {}

    Output(const TileId &tileId, const Input::list &input)
        : tileId(tileId), source(input) {}

    operator bool() const {
        return mesh || atlas || navtile;
    }

    const Mesh* getMesh() const { return mesh ? &*mesh : nullptr; }
    const opencv::RawAtlas* getAtlas() const {
        return atlas ? &*atlas : nullptr;
    }
    const NavTile* getNavtile() const { return navtile ? &*navtile : nullptr; }

    Mesh& forceMesh();
    opencv::RawAtlas& forceAtlas();

    bool derived(std::size_t index) const {
        if (index >= source.size()) { return false; }
        return (source[index].tileId().lod != tileId.lod);
    }

    bool fullyDerived() const {
        for (const auto &src : source) {
            if (src.tileId().lod == tileId.lod) { return false; }
        }
        return  true;
    }
};

/** Various merging constraints.
 */
class MergeConstraints {
public:
    MergeConstraints(bool generable = false)
        : generable_(generable)
    {}

    virtual ~MergeConstraints() {}

    /** Called when possible sources collected. Merge returns immediately on
     *  false.
     */
    bool generable() const { return generable_; }

    /** Called when exact sources are identified. Merge returns immediately on
     *  false. By default returns true.
     */
    virtual bool feasible(const Output &result) const;

private:
    bool generable_;
};

/** Generates new tile from given source and parent source fallback.
 *
 * Source and parent source inputs are merged together using their id's.
 *
 * If dummy flag is set only merged list of sources is returned.
 *
 * \param tileId tile's ID
 * \param nodeInfo node info
 * \param source list of current tile's sources
 * \param parentSource list of parent tile's sources
 * \param constraints various merging constraints
 */
Output mergeTile(const TileId &tileId
                 , const NodeInfo &nodeInfo
                 , const Input::list &source
                 , const Input::list &parentSource
                 , const MergeConstraints &constraints);

// inlines

inline bool MergeConstraints::feasible(const Output &) const { return true; }

} } } // namespace vadstena::vts::merge

#endif // vadstena_libs_vts_merge_hpp_included_
