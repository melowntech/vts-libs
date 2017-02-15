#ifndef vts_heightmap_hpp_included_
#define vts_heightmap_hpp_included_

#include <boost/filesystem/path.hpp>

#include <opencv2/core/core.hpp>

#include "geo/geodataset.hpp"

#include "../registry.hpp"
#include "./basetypes.hpp"
#include "./opencv/navtile.hpp"
#include "./meshopinput.hpp"

namespace vtslibs { namespace vts {

/** Missing delegation ctor workaround.
 */
struct HeightMapBase {
    const registry::ReferenceFrame *referenceFrame_;
    math::Size2 tileSize_;
    math::Size2 tileGrid_;
    Lod lod_;
    TileRange tileRange_;
    math::Size2 sizeInTiles_;
    math::Size2 sizeInPixels_;
    cv::Mat pane_;
    std::string srs_; // registry srs
    math::Extents2 worldExtents_;

    HeightMapBase(const registry::ReferenceFrame &referenceFrame
                  , Lod lod, TileRange tileRange);
};

class HeightMap : private HeightMapBase {
public:
    class Accumulator;

    /** Heightmap generation constructor.
     */
    HeightMap(Accumulator &&accumulator
              , const registry::ReferenceFrame &referenceFrame
              , double dtmExtractionRadius);

    /** Existing heightmap warping constructo.
     */
    HeightMap(const TileId &tileId, const MeshOpInput::list &source
              , const registry::ReferenceFrame &referenceFrame);

    math::Size2 size() const { return sizeInPixels_; };

    bool empty() const { return math::empty(sizeInPixels_); };

    /** Resizes this heightmap.
     *  (lod < lod_): heightmap is shrinked to fit tiles at given LOD
     *  (lod == lod_) no-op
     *  (lod > lod_): error
     */
    void resize(Lod lod);

    /** Warps heightmap to given node.
     *  Works for single destination tile.
     */
    void warp(const NodeInfo &nodeInfo);

    /** Generic warper.
     */
    void warp(const registry::ReferenceFrame &referenceFrame
              , Lod lod, const TileRange &tileRange);

    /** Returns navtile for given tile.
     *  Throws when tileId.lod != lod_.
     */
    NavTile::pointer navtile(const TileId &tileId) const;

    /** Dump heightmap as grayscale image. Masked-out pixels are set to black.
     */
    void dump(const boost::filesystem::path &filename) const;

    /** Best position inside heightmap
     */
    struct BestPosition;

    BestPosition bestPosition() const;
};

/** Best position inside heightmap
 */
struct HeightMap::BestPosition {
    /** Position in navigation SRS.
     */
    math::Point3 location;

    /** Vertical extent computed in a way that whole valid area is in view.
     */
    double verticalExtent;
};

/** Helper class. HeightMap can access internals.
 */
class HeightMap::Accumulator {
public:
    Accumulator(Lod lod);

    cv::Mat& tile(const TileId &tileId);

    const math::Size2& tileSize() const { return tileSize_; }

    typedef TileRange::point_type Index;

private:
    friend class HeightMap;

    Lod lod_;
    math::Size2 tileSize_;
    typedef std::map<Index, cv::Mat> Tiles;
    Tiles tiles_;
    TileRange tileRange_;
};

/** DTMize geodataset. Applies morphological opening `count` times in each
 *  direction.
 *
 *  Each iteration affects 1-pixel neighbourhood around each pixel.
 *
 * \param dataset geo dataset to be dtmized
 * \param count number of filter passes
 */
void dtmize(geo::GeoDataset &dataset, const math::Size2 &count);

} } // namespace vtslibs::vts

#endif // vts_heightmap_hpp_included_
