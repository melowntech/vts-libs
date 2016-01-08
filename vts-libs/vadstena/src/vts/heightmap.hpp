#ifndef vts_heightmap_hpp_included_
#define vts_heightmap_hpp_included_

#include <opencv2/core/core.hpp>

#include "vts-libs/vts/basetypes.hpp"

namespace vts = vadstena::vts;

class HeightMap {
public:
    class Accumulator;

    typedef vts::TileRange::point_type Index;

    HeightMap(Accumulator &&accumulator);

    void filter(int kernelSize = 3, int count = 3);

    math::Size2 size() const { return sizeInPixels_; };

private:
    math::Size2 tileSize_;
    vts::TileRange tileRange_;
    math::Size2 sizeInTiles_;
    math::Size2 sizeInPixels_;
    std::vector<cv::Mat> pane_;
};

/** Helper class. HeightMap can access internals.
 */
class HeightMap::Accumulator {
public:
    Accumulator();

    cv::Mat& tile(const vts::TileId &tileId);

    const math::Size2& tileSize() const { return tileSize_; }

private:
    cv::Mat* findTile(const HeightMap::Index &index);

    friend class HeightMap;
    math::Size2 tileSize_;
    typedef std::map<HeightMap::Index, cv::Mat> Pane;
    Pane pane_;
    vts::TileRange tileRange_;
    math::Size2 sizeInTiles_;
};

#endif // vts_heightmap_hpp_included_
