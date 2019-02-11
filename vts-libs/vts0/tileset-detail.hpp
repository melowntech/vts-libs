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
#include <new>
#include <queue>
#include <set>

#include "dbglog/dbglog.hpp"
#include "utility/progress.hpp"
#include "jsoncpp/json.hpp"

#include "../vts0.hpp"
#include "tileindex.hpp"
#include "io.hpp"
#include "tileop.hpp"
#include "driver.hpp"

namespace vtslibs { namespace vts0 {

typedef std::map<TileId, MetaNode> Metadata;

typedef std::set<TileId> TileIdSet;

struct TileSet::Detail {
    Driver::pointer driver;

    // properties
    Properties savedProperties;  // properties as are on disk
    Properties properties;       // current properties
    bool propertiesChanged;      // marks whether properties have been changed

    TileIndex tileIndex;    // tile index that reflects state on the disk
    TileIndex metaIndex;    // metatile index that reflects state on the disk

    LodRange lodRange;            // covered lod range
    mutable Metadata metadata;    // all metadata as in-memory structure
    mutable TileIdSet loadedMetatiles; // marks that given tiles are loaded
    bool metadataChanged;         // marks whether metadata have been changed

    bool tx; // pending transaction?

    Detail(const Driver::pointer &driver);
    Detail(const Driver::pointer &driver, const CreateProperties &properties);

    ~Detail();

    void checkValidity() const;

    void check(const TileId &tileId) const;

    void checkTx(const std::string &action) const;

    void loadConfig();

    void saveConfig();

    void saveMetadata();

    void loadTileIndex();

    Tile getTile(const TileId &tileId) const;

    boost::optional<Tile> getTile(const TileId &tileId, std::nothrow_t) const;

    MetaNode setTile(const TileId &tileId, const Mesh &mesh
                     , const Atlas &atlas, const TileMetadata *metadata
                     , const boost::optional<double> &pixelSize = boost::none);

    MetaNode removeTile(const TileId &tileId);

    MetaNode* loadMetatile(const TileId &tileId) const;

    void loadMetatileFromFile
        (Metadata &metadata, const TileId &tileId
         , const MetaNodeNotify &notify = MetaNodeNotify())
        const;

    MetaNode* findMetaNode(const TileId &tileId) const;

    MetaNode setMetaNode(const TileId &tileId, const MetaNode& metanode);

    void setMetadata(const TileId &tileId, const TileMetadata& metadata);

    MetaNode& createVirtualMetaNode(const TileId &tileId);

    void updateTreeMetadata(const TileId &tileId);

    void updateTreeMetadata(const TileId &tileId, MetaNode &metanode);

    void updateTree(const TileId &tileId);

    void updateTree(const TileId &tileId, MetaNode &metanode);

    void flush();

    void purgeMetadata();

    void saveMetatiles(TileIndex &tileIndex, TileIndex &metaIndex) const;

    void begin(utility::Runnable *runnable);

    void commit();

    void rollback();

    void watch(utility::Runnable *runnable);

    void fixDefaultPosition(const list &tileSets);

    TileId parent(const TileId &tileId) const;

    /** Filters heightfield in area affected by bordering tiles of continuous
     *  area and bordering tiles of discrete tiles.
     *
     * Each entry in discrete list belongs to appropriate continuous list.
     *
     * \param continuous list of tile indices of continuous area
     * \param discrete list of tile indices of individual tiles
     * \param cutoff cut off period of CatmullRom2 (in both directions)
     */
    void filterHeightmap(const TileIndices &continuous
                         , const TileIndices *discrete = nullptr
                         , double cutoff = 2.0);

    Statistics stat() const;

    Detail& other(const TileSet::pointer &otherSet) {
        return otherSet->detail();
    }

    const Detail& other(const TileSet::pointer &otherSet) const {
        return otherSet->detail();
    }

    void clone(const Detail &src);

    void clone(const Detail &src, const CloneOptions::Filter &filter);
};


// inline stuff

inline TileId TileSet::Detail::parent(const TileId &tileId) const
{
    return vts0::parent(tileId);
}

inline void TileSet::Detail::checkValidity() const
{
    if (!driver) {
        LOGTHROW(err2, storage::NoSuchTileSet)
            << "Tile set <" << properties.id << "> was removed.";
    }
}

} } // namespace vtslibs::vts0
