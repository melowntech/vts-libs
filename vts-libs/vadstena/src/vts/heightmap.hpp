#ifndef vts_heightmap_hpp_included_
#define vts_heightmap_hpp_included_

#include <opencv2/core/core.hpp>

#include "vts-libs/vts/basetypes.hpp"

namespace vts = vadstena::vts;

class HeightMap {
public:
    class Accumulator;

    HeightMap(Accumulator &&accumulator);

    void dtmize(int count);

    math::Size2 size() const { return sizeInPixels_; };

private:
    math::Size2 tileSize_;
    math::Size2 tileGrid_;
    vts::TileRange tileRange_;
    math::Size2 sizeInTiles_;
    math::Size2 sizeInPixels_;
    cv::Mat pane_;
};

/** Helper class. HeightMap can access internals.
 */
class HeightMap::Accumulator {
public:
    Accumulator();

    cv::Mat& tile(const vts::TileId &tileId);

    const math::Size2& tileSize() const { return tileSize_; }

private:
    typedef vts::TileRange::point_type Index;
    cv::Mat* findTile(const Index &index);

    friend class HeightMap;
    math::Size2 tileSize_;
    typedef std::map<Index, cv::Mat> Tiles;
    Tiles tiles_;
    vts::TileRange tileRange_;
    math::Size2 sizeInTiles_;
};

#endif // vts_heightmap_hpp_included_
