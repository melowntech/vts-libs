#ifndef vts_heightmap_hpp_included_
#define vts_heightmap_hpp_included_

#include <boost/filesystem/path.hpp>

#include <opencv2/core/core.hpp>

#include "vts-libs/registry.hpp"
#include "vts-libs/vts/basetypes.hpp"
#include "vts-libs/vts/navtile.hpp"

namespace vts = vadstena::vts;
namespace vr = vadstena::registry;

class HeightMap {
public:
    class Accumulator;

    HeightMap(Accumulator &&accumulator
              , const vr::ReferenceFrame &referenceFrame
              , double dtmExtractionRadius);

    math::Size2 size() const { return sizeInPixels_; };

    /** Resizes this heightmap.
     *  (lod < lod_): heightmap is shrinked to fit tiles at given LOD
     *  (lod == lod_) no-op
     *  (lod > lod_): error
     */
    void resize(vts::Lod lod);

    /** Returns navtile for given tile.
     *  Throws when tileId.lod != lod_.
     */
    vts::NavTile::pointer navtile(const vts::TileId &tileId) const;

    /** Dump heightmap as grayscale image. Masked-out pixels are set to black.
     */
    void dump(const boost::filesystem::path &filename) const;

    /** Best position inside heightmap
     */
    struct BestPosition;

    BestPosition bestPosition() const;

private:
    math::Size2 calculateSizeInPixels(math::Size2 sizeInTiles) const;

    const vr::ReferenceFrame &referenceFrame_;
    math::Size2 tileSize_;
    math::Size2 tileGrid_;
    vts::Lod lod_;
    vts::TileRange tileRange_;
    math::Size2 sizeInTiles_;
    math::Size2 sizeInPixels_;
    cv::Mat pane_;
    math::Extents2 worldExtents_;
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
    Accumulator(vts::Lod lod);

    cv::Mat& tile(const vts::TileId &tileId);

    const math::Size2& tileSize() const { return tileSize_; }

private:
    typedef vts::TileRange::point_type Index;

    friend class HeightMap;

    vts::Lod lod_;
    math::Size2 tileSize_;
    typedef std::map<Index, cv::Mat> Tiles;
    Tiles tiles_;
    vts::TileRange tileRange_;
    math::Size2 sizeInTiles_;
};

#endif // vts_heightmap_hpp_included_
