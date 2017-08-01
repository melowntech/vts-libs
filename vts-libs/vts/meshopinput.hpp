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
 * \file vts/meshopinput.hpp
 * \author Vaclav Blazek <vaclav.blazek@citationtech.net>
 *
 * Mesh operation input.
 */

#ifndef vtslibs_vts_meshop_hpp_included_
#define vtslibs_vts_meshop_hpp_included_

#include <boost/optional.hpp>

#include "./basetypes.hpp"
#include "./tileop.hpp"
#include "./atlas.hpp"
#include "./opencv/navtile.hpp"
#include "./tileset.hpp"

namespace vtslibs { namespace vts {

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

    static math::Matrix4 sd2Coverage(const NodeInfo &nodeInfo);

    static math::Matrix4 coverage2Sd(const NodeInfo &nodeInfo);

    static math::Matrix4 coverage2Texture();

    /** Return owning tileset
     */
    const TileSet::Detail *owner() const { return owner_; }

    const std::string& name() const;

    const TileId& tileId() const { return tileId_; }

    Id id() const { return id_; }

    bool operator<(const MeshOpInput &o) const { return id_ < o.id_; }

    bool inMergeableRange() const {
        return in(mergeableRange_, tileId_.lod); }

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

    LodRange mergeableRange_;
};

} } // namespace vtslibs::vts

#endif // vtslibs_vts_meshop_hpp_included_
