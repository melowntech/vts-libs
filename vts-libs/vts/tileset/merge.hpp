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
 * \file vts/merge.hpp
 * \author Vaclav Blazek <vaclav.blazek@citationtech.net>
 *
 * Tile content merge.
 */

#ifndef vtslibs_vts_merge_hpp_included_
#define vtslibs_vts_merge_hpp_included_

#include <boost/optional.hpp>

#include "../basetypes.hpp"
#include "../tileop.hpp"
#include "../opencv/atlas.hpp"
#include "../opencv/navtile.hpp"
#include "../meshopinput.hpp"
#include "detail.hpp"

namespace vtslibs { namespace vts { namespace merge {

typedef std::vector<math::Points3d> Vertices3List;

/** Merge input.
 */
typedef MeshOpInput Input;

struct TileSource {
    // list of tiles this tile's mesh/atlas was generated from
    Input::list mesh;

    // source of navtile
    Input::list navtile;

    TileSource() {}
    TileSource(const Input::list &mesh, const Input::list &navtile)
        : mesh{mesh}, navtile{navtile} {}
};

/** Merge output.
 */
struct Output {
    TileId tileId;

    Mesh::pointer mesh;
    opencv::HybridAtlas::pointer atlas;
    opencv::NavTile::pointer navtile;
    GeomExtents geomExtents;

    // list of tiles this tile was generated from
    TileSource source;

    explicit Output(const TileId &tileId) : tileId(tileId) {}

    Output(const TileId &tileId, const Input &input
           , const Input::list &navtileInput)
        : tileId(tileId), source({input}, navtileInput) {}

    Output(const TileId &tileId, const Input::list &meshInput
           , const Input::list &navtileInput)
        : tileId(tileId), source(meshInput, navtileInput)
    {}

    operator bool() const {
        return mesh || atlas || navtile;
    }

    const Mesh* getMesh() const { return mesh ? mesh.get() : nullptr; }
    const opencv::HybridAtlas* getAtlas() const {
        return atlas ? atlas.get() : nullptr;
    }
    const NavTile* getNavtile() const {
        return navtile ? navtile.get() : nullptr;
    }

    /** Takes content as a tile
     *
     *  NB: all shared pointers in returned Tile point to data inside this
     *  Output instance. Do not use tile after this instance destruction!
     */
    Tile tile(int textureQuality);

    Mesh& forceMesh();
    opencv::HybridAtlas& forceAtlas();
    opencv::NavTile& forceNavtile();

    bool derived(std::size_t index) const {
        if (index >= source.mesh.size()) { return false; }
        return (source.mesh[index].tileId().lod != tileId.lod);
    }

    bool fullyDerived() const {
        for (const auto &src : source.mesh) {
            if (src.tileId().lod == tileId.lod) { return false; }
        }
        return  true;
    }

    bool derived(const Input &input) const {
        return (input.tileId().lod != tileId.lod);
    }
};

/** Various merging constraints.
 */
class MergeConstraints {
public:
    MergeConstraints(bool generable = false, bool generateNavtile = false)
        : generable_(generable), generateNavtile_(generateNavtile)
    {}

    virtual ~MergeConstraints() {}

    /** Called when possible sources collected. Merge returns immediately on
     *  false.
     */
    bool generable() const { return generable_; }

    /** Navtile is generable.
     */
    bool generateNavtile() const { return generateNavtile_; }

    /** Called when exact sources are identified. Merge returns immediately on
     *  false. By default returns true.
     */
    virtual bool feasible(const Output &result) const;

private:
    bool generable_;
    bool generateNavtile_;
};

/** Extra merge options. Use only in special cases.
 */
struct ExtraOptions {
    /** Meshes are stored in spatial division SRS already if true. Output mesh
     *  is kept in SDS as well.
     */
    bool meshesInSds;

    /** Single sourced merges nornally skip coverage checking. Setting this flag
     *  to true forces check even for single sourced tiles.
     */
    bool checkCoverageForSingleSources;

    ExtraOptions()
        : meshesInSds(false), checkCoverageForSingleSources(false)
    {}
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
                 , const TileSource &parentSource
                 , const MergeConstraints &constraints
                 , const MergeOptions &options
                 , const ExtraOptions &extraOptions = ExtraOptions());

// inlines

inline bool MergeConstraints::feasible(const Output &) const { return true; }

} } } // namespace vtslibs::vts::merge

#endif // vtslibs_vts_merge_hpp_included_
