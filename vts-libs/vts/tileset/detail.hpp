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
 * \file vts/tileset.hpp
 * \author Vaclav Blazek <vaclav.blazek@citationtech.net>
 *
 * Tile set access.
 */

#ifndef vtslibs_vts_tileset_detail_hpp_included_
#define vtslibs_vts_tileset_detail_hpp_included_

#include <boost/any.hpp>

#include "../tileset.hpp"
#include "../tileindex.hpp"
#include "driver.hpp"
#include "tilesetindex.hpp"
#include "metacache.hpp"
#include "extra.hpp"

namespace vtslibs { namespace vts {

typedef std::map<std::string, math::Extents2> SdsExtents;

class LoddedTexelSizeAggregator {
public:
    LoddedTexelSizeAggregator()
        : lr_(LodRange::emptyRange())
    {}

    TexelSizeAggregator& operator[](Lod lod) {
        if (lr_.empty()) {
            aggs_.emplace_back();
            lr_ = LodRange(lod);
        } else {
            while (lod < lr_.min) {
                aggs_.emplace(aggs_.begin());
                --lr_.min;
            }
            while (lod > lr_.max) {
                aggs_.emplace_back();
                ++lr_.max;
            }
        }

        return aggs_[lod - lr_.min];
    }

private:
    LodRange lr_;
    std::vector<TexelSizeAggregator> aggs_;
};

bool check(const SdsExtents &l, const SdsExtents &r);

struct TileNode {
    MetaTile::pointer metatile;
    const MetaNode *metanode;

    TileNode() : metatile(), metanode() {}
    TileNode(const MetaTile::pointer &metatile, const MetaNode *metanode)
        : metatile(metatile), metanode(metanode)
    {}

    void update(const TileId &tileId, const MetaNode &mn)
    {
        metatile->update(tileId, mn);
    }

    void set(const TileId &tileId, const MetaNode &mn)
    {
        metatile->set(tileId, mn);
    }

    operator bool() const { return metanode; }
};

/** Driver that implements physical aspects of tile set.
 */
struct TileSet::Detail
{
public:
    registry::ReferenceFrame referenceFrame;

private:
    mutable tileset::Index tsi_;
    tileset::Index *driverTsi_;

public:
    bool readOnly;

    Driver::pointer driver;

    Properties properties;       // current properties
    mutable bool propertiesChanged; // marks that properties have been changed
    mutable bool metadataChanged;   // marks that metadata have been changed
    bool changed() const { return metadataChanged || propertiesChanged; }

    mutable std::unique_ptr<MetaCache> metaTiles;

    tileset::Index &tsi;
    /** Index of existing tiles.
     */
    TileIndex &tileIndex;

    LodRange lodRange;

    SdsExtents sdsExtents;
    LoddedTexelSizeAggregator texelSizeAggregator;

    Detail(const Driver::pointer &driver);
    Detail(const Driver::pointer &driver
           , const TileSet::Properties &properties);
    ~Detail();

    void loadConfig();

    void saveConfig();

    void watch(utility::Runnable *runnable);

    void checkValidity() const;

    MetaTile::pointer findMetaTile(const TileId &tileId, bool addNew = false) const;
    TileNode findNode(const TileId &tileId, bool addNew = false) const;
    const MetaNode* findMetaNode(const TileId &tileId) const;

    int getMetaTileVersion(const TileId &tileId) const;

    void loadTileIndex();
    void saveTileIndex();

    void setTile(const TileId &tileId, const Tile &tile
                 , const NodeInfo *nodeInfo = nullptr);

    void setTile(const TileId &tileId, const TileSource &tile
                 , const NodeInfo *nodeInfo = nullptr);

    void setNavTile(const TileId &tileId, const NavTile &navtile);

    void setSurrogateValue(const TileId &tileId, float value);

    std::uint8_t metaOrder() const;
    TileId metaId(TileId tileId) const;

    void save(const OStream::pointer &os, const Mesh &mesh
              , const Atlas *atlas = nullptr) const;
    void save(const OStream::pointer &os, const Atlas &atlas) const;
    void save(const OStream::pointer &os, const NavTile &navtile) const;

    void load(const IStream::pointer &os, Mesh &mesh) const;
    void load(const IStream::pointer &os, Atlas &atlas) const;
    void load(const NavTile::HeightRange &heightRange
              , const IStream::pointer &os, NavTile &navtile) const;
    void load(const IStream::pointer &os, MeshMask &meshMask) const;

    MetaTile::pointer addNewMetaTile(const TileId &tileId) const;

    MetaTile::pointer loadMetaTileFor(const TileId &tileId) const;

    void updateNode(TileId tileId, const MetaNode &metanode
                    , TileIndex::Flag::value_type extraFlags = 0);

    bool exists(const TileId &tileId) const;
    Mesh getMesh(const TileId &tileId) const;
    MeshMask getMeshMask(const TileId &tileId, bool generate) const;
    void getAtlas(const TileId &tileId, Atlas &atlas) const;
    void getNavTile(const TileId &tileId, NavTile &navtile) const;

    /** Trusts that findMetaNode(tileId) yields the same value as node.
     */
    Mesh getMesh(const TileId &tileId, const MetaNode *node) const;
    MeshMask getMeshMask(const TileId &tileId, const MetaNode *node
                         , bool generate) const;
    void getAtlas(const TileId &tileId, Atlas &atlas
                  , const MetaNode *node) const;
    void getNavTile(const TileId &tileId, NavTile &navtile
                    , const MetaNode *node) const;

    /** Trusts that tileindex flags yield the same information.
     */
    Mesh getMesh(const TileId &tileId, TileIndex::Flag::value_type flags)
        const;
    void getAtlas(const TileId &tileId, Atlas &atlas
                  , TileIndex::Flag::value_type flags) const;

    TileSource getTileSource(const TileId &metaId) const;

    void flush();
    void saveMetadata();
    void emptyCache() const;

    MapConfig mapConfig(bool includeExtra) const;

    static MapConfig mapConfig(const Driver &driver, bool includeExtra);

    MeshTilesConfig meshTilesConfig(bool includeExtra) const;

    static MeshTilesConfig meshTilesConfig(const Driver &driver
                                           , bool includeExtra);

    ExtraTileSetProperties loadExtraConfig() const;
    static ExtraTileSetProperties loadExtraConfig(const Driver &driver);

    registry::RegistryBase loadRegistry() const;
    static registry::RegistryBase loadRegistry(const Driver &driver);

    bool fullyCovered(const TileId &tileId) const;

    TileIndex::Flag::value_type extraFlags(const TileId &tileId) const;

    void setPosition(const registry::Position &position);
    void addCredits(const registry::IdSet &credits);
    void addBoundLayers(const registry::IdSet &boundLayers);

    double texelSize() const;

    void markInfluencedTile(const TileId &tileId);

private:
    void updateProperties(Lod lod, const MetaNode &metanode
                          , const MetaNode &oldMetanode);
    void updateProperties(const NodeInfo &nodeInfo);
    void updateProperties(const Mesh &mesh);
};

inline void TileSet::Detail::checkValidity() const {
    if (!driver) {
        LOGTHROW(err2, storage::NoSuchTileSet)
            << "Tile set <" << properties.id << "> was removed.";
    }
}

inline std::uint8_t TileSet::Detail::metaOrder() const
{
    return referenceFrame.metaBinaryOrder;
}

inline TileId TileSet::Detail::metaId(TileId tileId) const
{
    tileId.x &= ~((1 << referenceFrame.metaBinaryOrder) - 1);
    tileId.y &= ~((1 << referenceFrame.metaBinaryOrder) - 1);
    return tileId;
}

inline const MetaNode* TileSet::Detail::findMetaNode(const TileId &tileId)
    const
{
    // metanode is nullptr if non-existent
    return findNode(tileId).metanode;
}

} } // namespace vtslibs::vts

#endif // vtslibs_vts_tileset_detail_hpp_included_
