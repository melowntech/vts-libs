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
#include <boost/format.hpp>

#include "../tileset-detail.hpp"
#include "../merge.hpp"
#include "dump.hpp"
#include "../tileindex-io.hpp"
#include "../tileopext.hpp"

namespace vtslibs { namespace vts0 {

namespace fs = boost::filesystem;

namespace {
    const char *TILEINDEX_DUMP_ROOT("TILEINDEX_DUMP_ROOT");
} // namespace

const char* getDumpDir()
{
    return std::getenv(TILEINDEX_DUMP_ROOT);
}

/** This simple class allows access to the "detail" implementation of a tile
 * set.
 */
struct TileSet::Accessor
{
    static Detail& detail(TileSet &ts) { return ts.detail(); }
    static const Detail& detail(const TileSet &ts) { return ts.detail(); }
};

namespace {

LodRange range(const TileSet::list &sets)
{
    if (sets.empty()) {
        // empty set
        return LodRange::emptyRange();
    }

    // initialize with invalid range
    LodRange r(LodRange::emptyRange());
    for (const auto &set : sets) {
        r = unite(r, TileSet::Accessor::detail(*set).lodRange);
    }
    return r;
}

inline void tileIndicesImpl(TileIndices &indices, const TileSet::list &set)
{
    // get all tile indices
    for (const auto &ts : set) {
        auto &detail(TileSet::Accessor::detail(*ts));
        // we need fresh index
        detail.flush();
        indices.push_back(&detail.tileIndex);
    }
}

template <typename ...Args>
inline void tileIndicesImpl(TileIndices &indices, const TileSet::list &set
                            , Args &&...rest)
{
    tileIndicesImpl(indices, set);
    tileIndicesImpl(indices, std::forward<Args>(rest)...);
}

template <typename ...Args>
inline TileIndices tileIndices(Args &&...args)
{
    TileIndices indices;
    tileIndicesImpl(indices, std::forward<Args>(args)...);
    return indices;
}

inline void dump(const boost::filesystem::path &dir, const TileSet::list &set)
{
    int i(0);
    for (const auto &s : set) {
        auto &detail(TileSet::Accessor::detail(*s));
        // we need fresh index
        detail.flush();
        LOG(info2) << "Dumping <" << s->getProperties().id << ">.";

        auto path(dir / str(boost::format("%03d") % i));
        dumpAsImages(path, detail.tileIndex);
        detail.tileIndex.save(path / "raw.bin");
        ++i;
    }
}

inline void dump(const char *root, const boost::filesystem::path &dir
                 , const TileSet::list &set)
{
    if (!root) { return; }
    dump(root / dir, set);
}

} // namespace

void dumpTileIndex(const char *root, const fs::path &name
                   , const TileIndex &index)
{
    if (!root) { return; }

    const auto path(root / name);

    dumpAsImages(path, index);
    index.save(path / "raw.bin");
}

namespace {

struct Merger {
    Merger(TileSet::Detail &self, const TileIndex &world
           , const TileIndex &generate, const TileIndex *remove
           , const TileSet::list &src)
        : self(self), world(world), generate(generate), remove(remove)
        , src(src), progress(generate.count() + (remove ? remove->count() : 0))
    {
        for (const auto &ts : src) {
            affectedTiles.insert
                ({ ts.get(), std::make_shared<AffectedArea>
                        (self.other(ts).tileIndex) });
        }
    }

    /** Merge subtree starting at index.
     *  Calls itself recursively.
     *
     * NB Remove tile set must have same dimensions as generate tile set (if
     * non-null).
     */
    void mergeSubtree(const TileId &index
                      , const MergeInput::list &parentIncidentTiles
                      = MergeInput::list()
                      , int quadrant = -1
                      , bool parentGenerated = false);

    /** Generates new tile as a merge of tiles from other tilesets.
     */
    Tile generateTile(const TileId &tileId
                      , const MergeInput::list &parentIncidentTiles
                      , MergeInput::list &incidentTiles
                      , int quadrant);

    void filterHeightmap();

    TileSet::Detail &self;
    const TileIndex &world;
    const TileIndex &generate;
    const TileIndex *remove;
    const TileSet::list &src;

    utility::Progress progress;

    struct AffectedArea {
        TileIndex continuous;
        TileIndex discrete;

        typedef std::map<const TileSet*, std::shared_ptr<AffectedArea> >
            map;

        AffectedArea(const TileIndex &ti)
            : continuous(ti, TileIndex::ShallowCopy{})
            , discrete(ti, TileIndex::ShallowCopy{})
        {}
    };

    AffectedArea::map affectedTiles;
};

} // namespace

void TileSet::mergeIn(const list &kept, const list &update)
{
    // TODO: check validity of kept and update
    detail().checkValidity();

    // TODO: check tile compatibility

    LOG(info3) << "(merge-in) Calculating generate set.";

    const auto *dumpRoot(getDumpDir());

    dump(dumpRoot, "update", update);

    // calculate storage update (we need to know lod range of kept sets to
    // ensure all indices cover same lod range)
    auto tsUpdate(unite(tileIndices(update), range(kept)));
    if (tsUpdate.empty()) {
        LOG(warn3) << "(merge-in) Nothing to merge in. Bailing out.";
        return;
    }

    dumpTileIndex(dumpRoot, "tsUpdate", tsUpdate);
    tsUpdate.growDown();
    dumpTileIndex(dumpRoot, "tsUpdate-gd", tsUpdate);

    // calculate storage post state
    auto tsPost(unite(tileIndices(update, kept), tsUpdate));
    dumpTileIndex(dumpRoot, "tsPost", tsPost);
    tsPost.growUp();
    dumpTileIndex(dumpRoot, "tsPost-gu", tsPost);

    // calculate storage pre state
    auto tsPre(unite(tileIndices(kept), tsUpdate));
    dumpTileIndex(dumpRoot, "tsPre", tsPre);
    tsPre.growUp().invert();
    dumpTileIndex(dumpRoot, "tsPre-gu-inv", tsPre);

    LOG(info2) << "(merge-in) down(tsUpdate): " << tsUpdate;
    LOG(info2) << "(merge-in) up(tsPost): " << tsPost;
    LOG(info2) << "(merge-in) inv(up(tsPre)): " << tsPre;

    auto generate(intersect(tsPost, unite(tsUpdate, tsPre)));
    dumpTileIndex(dumpRoot, "generate", generate);

    LOG(info2) << "(merge-in) generate: " << generate;

    if (generate.empty()) {
        LOG(warn3) << "(merge-in) Nothing to generate. Bailing out.";
        return;
    }

    // make world
    TileIndex world(generate);
    world.makeComplete();
    dumpTileIndex(dumpRoot, "world", world);

    list all(kept.begin(), kept.end());
    all.insert(all.end(), update.begin(), update.end());

    Merger merger(detail(), world, generate, nullptr, all);

    LOG(info3)
        << "(merge-in) Generate set calculated. "
        << "About to process " << merger.progress.total() << " tiles.";

    auto lod(generate.minLod());
    traverse(lod, generate, [&](const TileId &tileId)
    {
        merger.mergeSubtree(tileId);
    });

    LOG(info3) << "(merge-in) Tile sets merged in.";

    // center default position if not inside tileset
    detail().fixDefaultPosition(all);

    merger.filterHeightmap();
}


namespace {

Tile Merger::generateTile(const TileId &tileId
                          , const MergeInput::list &parentIncidentTiles
                          , MergeInput::list &incidentTiles
                          , int quadrant)
{
    auto ts(tileSize(self.properties, tileId.lod));

    // Fetch tiles from other source.
    MergeInput::list tiles;
    for (const auto &ts : src) {
        if (auto t = self.other(ts).getTile(tileId, std::nothrow)) {
            tiles.push_back(MergeInput(t.get(), ts.get(), tileId ));
        }
    }

    // optimization
    if (parentIncidentTiles.empty()) {
        // no parent data
        if (tiles.empty()) {
            // no data -> remove tile and return empty tile
            self.removeTile(tileId);
            return {};
        } else if ((tiles.size() == 1)) {
            // just one single tile without any fallback
            // NB: copy pixelSize
            auto tile(tiles.front().tile());
            //auto tile(tiles2.front());
            tile.metanode
                = self.setTile(tileId, tile.mesh, tile.atlas, &tile.metanode
                               , tile.metanode.pixelSize[0][0]);
            incidentTiles = tiles;
            return tile;
        }

        // more tiles => must merge
    }

    // we have to merge tiles
    auto tile(merge(tileId, ts, tiles, quadrant
                    , parentIncidentTiles, incidentTiles));

    if (tile.singleSource()) {
        affectedTiles[tile.sources.front()]->continuous.set(tileId);
    } else if (tile.multiSource()) {
        affectedTiles[tile.sources.front()]->discrete.set(tileId);
    }

    // TODO: if single sourced => clone original tile instead
    tile.metanode
        = self.setTile(tileId, tile.mesh, tile.atlas, &tile.metanode
                       , tile.pixelSize());

    return tile;
}

void Merger::mergeSubtree(const TileId &tileId
                          , const MergeInput::list &parentIncidentTiles
                          , int quadrant, bool parentGenerated)
{
    if (!world.exists(tileId)) {
        // no data below
        return;
    }

    // tile generated in this run (empty and invalid by default)
    Tile tile;
    MergeInput::list incidentTiles;

    // should this tile be generated?
    auto g(generate.exists(tileId));
    auto r(remove && remove->exists(tileId));
    if (g) {
        LOG(info2) << "(merge-in) Processing tile "
                   << tileId << ".";

        bool thisGenerated(false);
        if (!parentGenerated) {
            if (auto t = self.getTile(self.parent(tileId), std::nothrow)) {
                // no parent was generated and we have sucessfully loaded parent
                // tile from existing content as a fallback tile!

                // TODO: since this is merge result its information can be
                // outdated
                MergeInput::list loadedParentTiles;
                loadedParentTiles.push_back
                    (MergeInput(t.get(), nullptr, tileId));

                quadrant = child(tileId);
                tile = generateTile(tileId, loadedParentTiles
                                    , incidentTiles, quadrant);
                thisGenerated = true;
            }
        }

        if (!thisGenerated) {
            // regular generation
            tile = generateTile(tileId, parentIncidentTiles
                                , incidentTiles, quadrant);
        }
        (++progress).report(utility::Progress::ratio_t(5, 1000), "(merge) ");
    } else if (r) {
        LOG(info2) << "(merge-out) Processing tile "
                   << tileId << ".";
        self.removeTile(tileId);
        (++progress).report(utility::Progress::ratio_t(5, 1000), "(merge) ");
    }

    // can we go down?
    if (tileId.lod >= generate.maxLod()) {
        // no way down
        return;
    }

    // OK, process children
    quadrant = 0;
    for (const auto &child : children(tileId)) {
        mergeSubtree(child, incidentTiles, quadrant++, g);
    }
}

void Merger::filterHeightmap()
{
    TileIndices continuous;
    TileIndices discrete;
    for (const auto &info : affectedTiles) {
        LOG(info2)
            << "Using tile index from <" << info.first->getProperties().id
            << ">.";
        continuous.push_back(&info.second->continuous);
        discrete.push_back(&info.second->discrete);
    }

    // TODO: implement me
    self.filterHeightmap(continuous, &discrete);
}

} // namespace

void TileSet::Detail::fixDefaultPosition(const list &tileSets)
{
    double maxHeight(400);
    for (const auto &ts : tileSets) {
        maxHeight = std::max(ts->getProperties().defaultPosition(2)
                             , maxHeight);
    }

    properties.defaultPosition
        = tileSets.front()->getProperties().defaultPosition;
    properties.defaultPosition(2) = maxHeight;
}

void TileSet::paste(const list &update)
{
    TileIndices changed;

    const auto thisId(getProperties().id);
    auto &det(detail());

    const utility::Progress::ratio_t reportRatio(1, 100);
    for (const auto &src : update) {
        const auto id(src->getProperties().id);
        const auto name(str(boost::format("Pasting <%s> into <%s> ")
                            % id % thisId));

        LOG(info2) << "Copying all tiles from <" << id << ">.";
        auto &sdet(src->detail());

        utility::Progress progress(sdet.tileIndex.count());

        // process all tiles
        traverseTiles(sdet.tileIndex, [&](const TileId &tileId)
        {
            const auto *metanode(sdet.findMetaNode(tileId));
            if (!metanode) {
                LOG(warn2)
                    << "Cannot find metanode for tile " << tileId << "; "
                    << "skipping.";
                return;
            }

            // copy mesh and atlas
            for (auto type : { TileFile::mesh, TileFile::atlas }) {
                copyFile(sdet.driver->input(tileId, type)
                         , det.driver->output(tileId, type));
            }

            det.setMetaNode(tileId, *metanode);
            (++progress).report(reportRatio, name);
        });

        changed.push_back(&sdet.tileIndex);
    }

    // filter heightmap in bordering tiles in all pasted tile sets
    det.filterHeightmap(changed);
}

} } // namespace vtslibs::vts0
