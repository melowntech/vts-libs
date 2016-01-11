#include "dbglog/dbglog.hpp"

#include "vts-libs/vts/navtile.hpp"

#include "./heightmap.hpp"

namespace def {

float Infinity(std::numeric_limits<float>::infinity());
float InvalidHeight(Infinity);

} // namespace def

HeightMap::Accumulator::Accumulator()
    : tileSize_(vts::NavTile::size().width
                , vts::NavTile::size().width)
    , tileRange_(math::InvalidExtents{})
{
}

cv::Mat* HeightMap::Accumulator::findTile(const Index &index)
{
    auto ftiles(tiles_.find(index));
    if (ftiles == tiles_.end()) { return nullptr; }
    return &ftiles->second;
}

cv::Mat& HeightMap::Accumulator::tile(const vts::TileId &tileId)
{
    Index index(tileId.x, tileId.y);
    if (auto *tile = findTile(index)) {
        return *tile;
    }

    // create new tile (all values set to -oo
    auto res
        (tiles_.insert
         (Tiles::value_type
          (index, cv::Mat
           (tileSize_.height, tileSize_.width, CV_32F
            , cv::Scalar(def::InvalidHeight)))));
    update(tileRange_, index);

    auto s(math::size(tileRange_));
    sizeInTiles_.width = 1 + s.width;
    sizeInTiles_.height = 1 + s.height;
    return res.first->second;
}

HeightMap::HeightMap(Accumulator &&a)
    : tileSize_(a.tileSize_)
    , tileGrid_(tileSize_.width - 1, tileSize_.height - 1)
    , tileRange_(a.tileRange_)
    , sizeInTiles_(a.sizeInTiles_)
    , sizeInPixels_(1 + sizeInTiles_.width * tileGrid_.width
                    , 1 + sizeInTiles_.height * tileGrid_.height)
    , pane_(sizeInPixels_.height, sizeInPixels_.width, CV_32F
            , cv::Scalar(def::InvalidHeight))
{
    LOG(info4) << "Copying heightmaps from tiles into one pane.";
    for (const auto &item : a.tiles_) {
        // copy tile in proper place
        Accumulator::Index offset(item.first - tileRange_.ll);
        offset(0) *= tileGrid_.width;
        offset(1) *= tileGrid_.height;
        cv::Mat tile(pane_, cv::Range(offset(1), offset(1) + tileSize_.height)
                     , cv::Range(offset(0), offset(0) + tileSize_.width));
        item.second.copyTo(tile);
    }

    // drop original pane
    a.tiles_.clear();
}

namespace {

template <typename Operator>
class Morphology {
public:
    Morphology(cv::Mat &data, cv::Mat &tmp, int kernelSize)
        : in_(data), tmp_(tmp)
    {
        tmp_ = cv::Scalar(def::InvalidHeight);
        run(kernelSize);
        std::swap(in_, tmp_);
    }

private:
    void run(int kernelSize);

    cv::Mat &in_;
    cv::Mat &tmp_;
};

template <typename Operator>
void Morphology<Operator>::run(int kernelSize)
{
    kernelSize /= 2;

    for (int y(0); y != in_.rows; ++y) {
        for (int x(0); x != in_.cols; ++x) {

            Operator op;

            for (int j = -kernelSize; j <= kernelSize; ++j) {
                const int yy(y + j);
                if ((yy < 0) || (yy >= in_.rows)) { continue; }

                for (int i = -kernelSize; i <= kernelSize; ++i) {
                    const int xx(x + i);
                    if ((xx < 0) || (xx >= in_.cols)) { continue; }

                    op(in_.template at<float>(yy, xx));
                }
            }

            // store only valid value
            if (!op.valid()) { continue; }

            // store
            tmp_.template at<float>(y, x) = op.get();
        }
    }
}

class Erosion {
public:
    Erosion() : value_(def::Infinity) {}

    inline void operator()(float value) {
        if (value != def::InvalidHeight) {
            value_ = std::min(value_, value);
        }
    }

    inline bool valid() const { return value_ != def::Infinity; }
    inline float get() const { return value_; }

private:
    float value_;
};

class Dilation {
public:
    inline Dilation() : value_(-def::Infinity) {}

    inline void operator()(float value) {
        if (value != def::InvalidHeight) {
            value_ = std::max(value_, value);
        }
    }

    inline bool valid() const { return value_ != -def::Infinity; }
    inline float get() const { return value_; }

private:
    float value_;
};

} // namespace

void HeightMap::dtmize(int count)
{
    LOG(info4) << "Generating DTM from heightmap ("
               << pane_.cols << "x" << pane_.rows << " pixels).";

    cv::Mat tmp(pane_.rows, pane_.cols, pane_.type());

    LOG(info2) << "Eroding heightmap (" << count << " iterations).";
    for (int c(0); c < count; ++c) {
        LOG(info1) << "Erosion iteration " << c << ".";
        Morphology<Erosion>(pane_, tmp, 2);
    }

    LOG(info2) << "Dilating heightmap (" << count << " iterations).";
    for (int c(0); c < count; ++c) {
        LOG(info1) << "Dilation iteration " << c << ".";
        Morphology<Dilation>(pane_, tmp, 2);
    }
}
