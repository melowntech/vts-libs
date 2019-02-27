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

#include <memory>
#include <vector>

#include <boost/optional.hpp>

#include "basetypes.hpp"
#include "tileop.hpp"
#include "opencv/atlas.hpp"
#include "opencv/navtile.hpp"
#include "tileset.hpp"

namespace vtslibs { namespace vts {

template <typename T>
std::shared_ptr<T> cloneEntity(const T &value)
{
    return std::make_shared<T>(value);
}

/** Mesh operation input.
 */
class MeshOpInput {
public:
    class DataSource;

    typedef int Id;

    /** Create meshop input.
     *
     *  \param id owner identifier; used for sorting purposes
     *  \param owner owner of the tile
     *  \param tileId tile identifier
     *  \param nodeInfo node info (fetched from tileset if null)
     *  \param lazy loads data on demand
     */
    MeshOpInput(Id id, std::shared_ptr<DataSource> owner
                , const TileId &tileId
                , const NodeInfo *nodeInfo = nullptr, bool lazy = true);

    /** Input is valid only if there is node with geometry
     */
    operator bool() const { return hasMesh(); }

    const MetaNode& node() const;
    const NodeInfo& nodeInfo() const { return nodeInfo_; }

    bool watertight() const;
    bool hasMesh() const;
    bool hasAtlas() const;
    bool hasNavtile() const;

    /** Returns mesh. Lazy load.
     */
    const Mesh& mesh() const;

    /** Returns atlas. Lazy load.
     */
    const opencv::HybridAtlas& atlas() const;

    /** Returns atlas. Lazy load.
     */
    const opencv::NavTile& navtile() const;

    typedef std::vector<MeshOpInput> list;

    static math::Matrix4 sd2Coverage(const NodeInfo &nodeInfo, int margin);

    static math::Matrix4 coverage2Sd(const NodeInfo &nodeInfo, int margin);

    static math::Matrix4 coverage2Texture(int margin);

    /** Return owning datasource
     */
    std::shared_ptr<DataSource> owner() const { return owner_; }

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
    std::shared_ptr<DataSource> owner_;
    TileIndex::Flag::value_type flags_;
    NodeInfo nodeInfo_;

    mutable bool nodeLoaded_;
    mutable const MetaNode *node_;
    mutable Mesh::pointer mesh_;
    mutable opencv::HybridAtlas::pointer atlas_;
    mutable opencv::NavTile::pointer navtile_;

    LodRange mergeableRange_;
};

class MeshOpInput::DataSource {
public:
    typedef std::shared_ptr<DataSource> pointer;
    typedef std::vector<pointer> list;

    DataSource(const TileSet::Properties &properties)
        : properties_(properties)
    {}

    virtual ~DataSource();

    /** Return datasource properties.
     */
    const TileSet::Properties& properties() const { return properties_; }

    /** Get tileindex flags for given tile.
     */
    TileIndex::Flag::value_type flags(const TileId &tileId) const;

    /** Find metanode for given tile.
     */
    const MetaNode* findMetaNode(const TileId &tileId) const;

    /** Get tile's mesh.
     */
    Mesh::pointer getMesh(const TileId &tileId
                          , TileIndex::Flag::value_type flags) const;

    /** Get tile's atlas.
     */
    opencv::HybridAtlas::pointer getAtlas(const TileId &tileId
                                          , TileIndex::Flag::value_type flags)
        const;

    /** Get tile's navtile.
     */
    opencv::NavTile::pointer getNavTile(const TileId &tileId
                                        , const MetaNode *node) const;

    /** Get node info for given tile.
     */
    NodeInfo nodeInfo(const TileId &tileId) const;

private:
    virtual TileIndex::Flag::value_type flags_impl(const TileId &tileId)
        const = 0;

    virtual const MetaNode* findMetaNode_impl(const TileId &tileId)
        const = 0;

    virtual Mesh::pointer getMesh_impl(const TileId &tileId
                                       , TileIndex::Flag::value_type flags)
        const = 0;

    virtual opencv::HybridAtlas::pointer
    getAtlas_impl(const TileId &tileId
                  , TileIndex::Flag::value_type flags) const = 0;

    virtual opencv::NavTile::pointer
    getNavTile_impl(const TileId &tileId, const MetaNode *node)
        const = 0;

    virtual NodeInfo nodeInfo_impl(const TileId &tileId) const = 0;

    /** Datasource properties.
     */
    const TileSet::Properties properties_;
};

MeshOpInput::DataSource::pointer
tilesetDataSource(const TileSet &tileset);

MeshOpInput::DataSource::pointer
tilesetDataSource(const TileSet::Detail &detail);

// inlines

inline TileIndex::Flag::value_type
MeshOpInput::DataSource::flags(const TileId &tileId) const
{
    return flags_impl(tileId);
}

inline const MetaNode*
MeshOpInput::DataSource::findMetaNode(const TileId &tileId) const
{
    return findMetaNode_impl(tileId);
}

inline Mesh::pointer
MeshOpInput::DataSource::getMesh(const TileId &tileId
                                 , TileIndex::Flag::value_type flags) const
{
    return getMesh_impl(tileId, flags);
}

inline opencv::HybridAtlas::pointer
MeshOpInput::DataSource::getAtlas(const TileId &tileId
                                  , TileIndex::Flag::value_type flags) const
{
    return getAtlas_impl(tileId, flags);
}

inline opencv::NavTile::pointer
MeshOpInput::DataSource::getNavTile(const TileId &tileId, const MetaNode *node)
    const
{
    return getNavTile_impl(tileId, node);
}

inline NodeInfo MeshOpInput::DataSource::nodeInfo(const TileId &tileId) const
{
    return nodeInfo_impl(tileId);
}
inline MeshOpInput::DataSource::pointer
tilesetDataSource(const TileSet &tileset)
{
    return tilesetDataSource(tileset.detail());
}

} } // namespace vtslibs::vts

#endif // vtslibs_vts_meshop_hpp_included_
