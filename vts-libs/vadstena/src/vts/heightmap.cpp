#include "dbglog/dbglog.hpp"

#include "vts-libs/vts/mesh.hpp"

#include "./heightmap.hpp"

namespace def {

float Infinity(std::numeric_limits<float>::infinity());
float InvalidHeight(-Infinity);

} // namespace def

HeightMap::Accumulator::Accumulator()
    : tileSize_(vts::Mesh::coverageSize().width
                , vts::Mesh::coverageSize().height)
    , tileRange_(math::InvalidExtents{})
{
}

cv::Mat* HeightMap::Accumulator::findTile(const HeightMap::Index &index)
{
    auto fpane(pane_.find(index));
    if (fpane == pane_.end()) { return nullptr; }
    return &fpane->second;
}

cv::Mat& HeightMap::Accumulator::tile(const vts::TileId &tileId)
{
    HeightMap::Index index(tileId.x, tileId.y);
    if (auto *tile = findTile(index)) {
        return *tile;
    }

    // create new tile (all values set to -oo
    auto res
        (pane_.insert
         (Pane::value_type
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
    , tileRange_(a.tileRange_)
    , sizeInTiles_(a.sizeInTiles_)
    , sizeInPixels_(sizeInTiles_.width * tileSize_.width
                    , sizeInTiles_.height * tileSize_.height)
    , pane_(area(sizeInTiles_))
{
    // copy tiles (NB: cv::Mat uses reference counting, no data are copied)
    unsigned int i(0);
    for (auto &t : pane_) {
        Index index(i % sizeInTiles_.width, i / sizeInTiles_.width);

        if (auto *tile = a.findTile(index)) {
            t = *tile;
        }

        ++i;
    }

    // drop original pane
    a.pane_.clear();

    LOG(info4) << "Tiles in hm: " << pane_.size() << ".";
    LOG(info4) << "Size in tiles: " << sizeInTiles_ << ".";
    LOG(info4) << "Size in pixels: " << sizeInPixels_ << ".";
}

namespace {

template <typename Operator>
class Morphology {
public:
    typedef std::vector<cv::Mat> Tiles;

    Morphology(Tiles &in, const math::Size2 &tileSize
               , const math::Size2 &sizeInTiles
               , int kernelSize)
        : in_(in), tileSize_(tileSize), sizeInTiles_(sizeInTiles)
        , sizeInPixels_(tileSize.width * sizeInTiles.width
                        , tileSize.height * sizeInTiles.height)
        , out_(in_.size())
    {
        run(kernelSize);
        in_.swap(out_);
    }

private:
    void run(int kernelSize);

    inline int tileIndex(int x, int y) {
        return ((x / tileSize_.width)
                + ((y / tileSize_.height) * sizeInTiles_.width));
    }

    math::Point2i tilePos(int x, int y) {
        return { x % tileSize_.width, y % tileSize_.height };
    }

    Tiles &in_;
    const math::Size2 tileSize_;
    const math::Size2 sizeInTiles_;
    const math::Size2 sizeInPixels_;
    Tiles out_;
};

template <typename Operator>
void Morphology<Operator>::run(int kernelSize)
{
    kernelSize /= 2;

    for (int y(0), ey(sizeInPixels_.height); y != ey; ++y) {
        for (int x(0), ex(sizeInPixels_.width); x != ex; ++x) {

            Operator op;

            for (int j = -kernelSize; j <= kernelSize; ++j) {
                const int yy(y + j);
                if ((yy < 0) || (yy >= sizeInPixels_.height)) { continue; }

                for (int i = -kernelSize; i <= kernelSize; ++i) {
                    const int xx(x + i);
                    if ((xx < 0) || (xx >= sizeInPixels_.width)) { continue; }

                    const auto ti(tileIndex(xx, yy));
                    const auto tp(tilePos(xx, yy));

                    auto &mat(in_[ti]);
                    if (mat.data) {
                        op(mat.template at<float>(tp(1), tp(0)));
                    }
                }
            }

            // store only valid value
            if (!op.valid()) { continue; }

            const auto ti(tileIndex(x, y));
            const auto tp(tilePos(x, y));
            auto &mat(out_[ti]);
            if (!mat.data) {
                // no tile yet -> create
                mat.create(tileSize_.height, tileSize_.width, CV_32F);
                mat = cv::Scalar(def::InvalidHeight);
            }

            // store
            mat.template at<float>(tp(1), tp(0)) = op.get();
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

void HeightMap::filter(int kernelSize, int count)
{
    (void) kernelSize;
    (void) count;

    for (int c(count); --c; ) {
        Morphology<Erosion>(pane_, tileSize_, sizeInTiles_, kernelSize);
    }

    for (int c(count); --c; ) {
        Morphology<Dilation>(pane_, tileSize_, sizeInTiles_, kernelSize);
    }
}
