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
#ifndef vtslibs_vts0_merge_hpp_included_
#define vtslibs_vts0_merge_hpp_included_

#include <boost/optional.hpp>

#include "types.hpp"
#include "tileset.hpp"

namespace vtslibs { namespace vts0 {

constexpr int MERGE_NO_FALLBACK_TILE = -100;

struct MergedTile : public Tile {
    MergedTile() = default;
    MergedTile(const Tile &tile) : Tile(tile) {}
    MergedTile(const Tile &tile, const TileSet &ts)
        : Tile(tile), sources{&ts}
    {}

    /** Returns prefered pixel size if available.
     */
    boost::optional<double> pixelSize() const;

    /** Tile data originates from single tile if true.
     */
    bool singleSource() const { return sources.size() == 1; }

    /** Tile data originates from multiple tiles if true.
     */
    bool multiSource() const { return sources.size() > 1; }

    /** List of tilesets this tile was merged from
     */
    std::vector<const TileSet*> sources;
};

class MergeInput
{
public:
    MergeInput(Tile tile, const TileSet *tileSet, TileId tileId)
        : tile_(tile), tileSet_(tileSet), tileId_(tileId)
    {
        srcFaceCount_  = tile.mesh.facets.size();
    }

    MergeInput( Tile tile, const TileSet *tileSet, TileId tileId
              , uint srcFaceCount )
        : tile_(tile), tileSet_(tileSet), tileId_(tileId)
        , srcFaceCount_(srcFaceCount)
    {}

    Tile& tile() { return tile_; }
    const Tile& tile() const { return tile_; }
    const TileSet& tileSet() const { return *tileSet_; }
    TileId tileId() { return tileId_ ;}
    const TileId tileId() const { return tileId_ ;}
    uint srcFaceCount() const { return srcFaceCount_; }

    typedef std::vector<MergeInput> list;
private:
    Tile tile_;
    const TileSet *tileSet_;
    TileId tileId_;
    uint srcFaceCount_;
};

/** Merge tiles based on their quality. Areas not covered by any tile in `tiles`
 *  is covered by `fallbackQuad` quadrant of `fallback` tile.
 *
 * Prerequisities:
 *     * all input tiles have dimension AxA
 *     * fallback tile has dimension 2Ax2A
 *     * output tile has dimension AxA
 *
 * \param tileId tile id (for diagnostics purposes only)
 * \param tileSize size of tile
 * \param mergeInput tiles from current lod to merge
 * \param quad which quadrant of fallback tile (or whole tile) to use:
 *            * 0: lower-left
 *            * 1: lower-right
 *            * 2: upper-left
 *            * 3: upper-right
 *            * other negative: ignore fallback tile
 * \param  ancestorTiles tiles from previous lods to merge
 * \param  incidentTiles is filled with all tiles incident 
 *         with current tile (clipped)
 * \return merged tile
 */
MergedTile merge( const TileId &tileId, const math::Size2f &tileSize
                , const MergeInput::list &mergeInput
                , int quad
                , const MergeInput::list &ancestorTiles
                , MergeInput::list &incidentTiles);

} } // namespace vtslibs::vts0

#endif // vtslibs_vts0_merge_hpp_included_
