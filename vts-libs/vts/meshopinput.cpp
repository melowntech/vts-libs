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

#include "tileset/detail.hpp"
#include "tileset/merge/support.hpp"

#include "meshopinput.hpp"

namespace vtslibs { namespace vts {

MeshOpInput::DataSource::~DataSource() {}

MeshOpInput::MeshOpInput(Id id, DataSource::pointer owner
                         , const TileId &tileId
                         , const NodeInfo *nodeInfo, bool lazy)
    : id_(id), tileId_(tileId), owner_(std::move(owner))
    , flags_(owner_->flags(tileId))
    , nodeInfo_(nodeInfo ? *nodeInfo : owner_->nodeInfo(tileId_))
    , nodeLoaded_(false), node_()
    , mergeableRange_(owner_->properties().lodRange)
{
    prepare(lazy);

    if (auto bottom = owner_->properties().mergeBottomLod) {
        mergeableRange_.max = bottom;
    }
}

bool MeshOpInput::loadNode() const
{
    if (!nodeLoaded_) {
        node_ = owner_->findMetaNode(tileId_);
        nodeLoaded_ = true;
    }
    return node_;
}

void MeshOpInput::prepare(bool lazy)
{
    if (lazy) { return; }

    // preload stuff if not lazy
    if (loadNode()) {
        if (hasMesh()) { mesh(); }
        if (hasAtlas()) { atlas(); }
        if (hasNavtile()) { navtile(); }
    }
}

const std::string& MeshOpInput::name() const
{
    return owner_->properties().id;
}

const MetaNode& MeshOpInput::node() const
{
    loadNode();
    return *node_;
}

bool MeshOpInput::watertight() const
{
    return (flags_ & TileIndex::Flag::watertight);
}

bool MeshOpInput::hasMesh() const
{
    return (flags_ & TileIndex::Flag::mesh);
}

bool MeshOpInput::hasAtlas() const
{
    return (flags_ & TileIndex::Flag::atlas);
}

bool MeshOpInput::hasNavtile() const
{
    return (flags_ & TileIndex::Flag::navtile);
}

const Mesh& MeshOpInput::mesh() const
{
    if (!mesh_) {
        mesh_ = owner_->getMesh(tileId_, flags_);
    }

    return *mesh_;
}

const opencv::HybridAtlas& MeshOpInput::atlas() const
{
    if (!atlas_) {
        atlas_ = owner_->getAtlas(tileId_, flags_);
    }

    return *atlas_;
}

const opencv::NavTile& MeshOpInput::navtile() const
{
    // navtile must have valid node to work properly
    if (!navtile_ && loadNode()) {
        navtile_ = owner_->getNavTile(tileId_, node_);
    }

    return *navtile_;
}

math::Matrix4 MeshOpInput::sd2Coverage(const NodeInfo &nodeInfo, int margin)
{
    return merge::geo2mask(nodeInfo.extents(), Mesh::coverageSize(), margin);
}

math::Matrix4 MeshOpInput::coverage2Sd(const NodeInfo &nodeInfo, int margin)
{
    return merge::mask2geo(nodeInfo.extents(), Mesh::coverageSize(), margin);
}

math::Matrix4 MeshOpInput::coverage2Texture(int margin)
{
    return merge::coverage2EtcTrafo(Mesh::coverageSize(), margin);
}


/** Tileset data source.
 */
class TsDataSource : public MeshOpInput::DataSource {
public:
    TsDataSource(const TileSet::Detail &detail)
        : DataSource(detail.properties), detail_(detail)
    {}

    virtual ~TsDataSource() {}

private:
    virtual TileIndex::Flag::value_type flags_impl(const TileId &tileId)
        const
    {
        return detail_.tileIndex.get(tileId);
    }

    virtual const MetaNode* findMetaNode_impl(const TileId &tileId)
        const
    {
        return detail_.findMetaNode(tileId);
    }

    virtual Mesh::pointer getMesh_impl(const TileId &tileId
                                       , TileIndex::Flag::value_type flags)
        const
    {
        return std::make_shared<Mesh>(detail_.getMesh(tileId, flags));
    }

    virtual opencv::HybridAtlas::pointer
    getAtlas_impl(const TileId &tileId, TileIndex::Flag::value_type flags)
        const
    {
        auto atlas(std::make_shared<opencv::HybridAtlas>());
        detail_.getAtlas(tileId, *atlas, flags);
        return atlas;
    }

    virtual opencv::NavTile::pointer
    getNavTile_impl(const TileId &tileId, const MetaNode *node)  const
    {
        auto navtile(std::make_shared<opencv::NavTile>());
        detail_.getNavTile(tileId, *navtile, node);
        return navtile;
    }

    virtual NodeInfo nodeInfo_impl(const TileId &tileId) const
    {
        return NodeInfo(detail_.referenceFrame, tileId);
    }

    const TileSet::Detail &detail_;
};

MeshOpInput::DataSource::pointer
tilesetDataSource(const TileSet::Detail &detail)
{
    return std::make_shared<TsDataSource>(detail);
}

} } // namespace vtslibs::vts
