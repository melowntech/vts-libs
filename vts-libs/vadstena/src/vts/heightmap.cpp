#include <cmath>

#include <boost/filesystem.hpp>
#include <boost/format.hpp>

#include <opencv2/highgui/highgui.hpp>

#include "dbglog/dbglog.hpp"

#include "imgproc/filtering.hpp"
#include "imgproc/reconstruct.hpp"

#include "vts-libs/vts/io.hpp"
#include "vts-libs/vts/tileop.hpp"
#include "vts-libs/vts/opencv/navtile.hpp"

#include "./heightmap.hpp"

namespace def {

float Infinity(std::numeric_limits<float>::infinity());
float InvalidHeight(Infinity);

const auto *DumpDir(::getenv("HEIGHTMAP_DUMP_DIR"));

} // namespace def

HeightMap::Accumulator::Accumulator(vts::Lod lod)
    : lod_(lod)
    , tileSize_(vts::NavTile::size().width
                , vts::NavTile::size().width)
    , tileRange_(math::InvalidExtents{})
{
}

cv::Mat& HeightMap::Accumulator::tile(const vts::TileId &tileId)
{
    if (tileId.lod != lod_) {
        LOGTHROW(err2, std::runtime_error)
            << "Unexpected tile ID " << tileId
            << ": this heightmap accumulator expects tiles at LOD "
            << lod_ << ".";
    }

    const Index index(tileId.x, tileId.y);

    const auto ftiles(tiles_.find(index));
    if (ftiles != tiles_.end()) {
        return ftiles->second;
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
            // skip invalid data
            if (in_.template at<float>(y, x) == def::InvalidHeight) {
                continue;
            }

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

void dtmize(cv::Mat &pane, int count)
{
    LOG(info3) << "Generating DTM from heightmap ("
               << pane.cols << "x" << pane.rows << " pixels).";

    cv::Mat tmp(pane.rows, pane.cols, pane.type());

    LOG(info2) << "Eroding heightmap (" << count << " iterations).";
    for (int c(0); c < count; ++c) {
        LOG(info1) << "Erosion iteration " << c << ".";
        Morphology<Erosion>(pane, tmp, 2);
    }

    LOG(info2) << "Dilating heightmap (" << count << " iterations).";
    for (int c(0); c < count; ++c) {
        LOG(info1) << "Dilation iteration " << c << ".";
        Morphology<Dilation>(pane, tmp, 2);
    }
}

void debugDump(const HeightMap &hm, const boost::filesystem::path &filename)
{
    if (!def::DumpDir) { return; }

    hm.dump(def::DumpDir / filename);
}

math::Extents2 worldExtents(vts::Lod lod, const vts::TileRange &tileRange
                            , const vr::ReferenceFrame &referenceFrame)
{
    vts::NodeInfo nodeLl
        (referenceFrame
         , vts::TileId(lod, tileRange.ll(0), tileRange.ur(1)));
    vts::NodeInfo nodeUr
        (referenceFrame
         , vts::TileId(lod, tileRange.ur(0), tileRange.ll(1)));

    return math::Extents2(nodeLl.node.extents.ll, nodeUr.node.extents.ur);
}

} // namespace

math::Size2 HeightMap::calculateSizeInPixels(math::Size2 sizeInTiles) const
{
    return { 1 + sizeInTiles.width * tileGrid_.width
            , 1 + sizeInTiles.height * tileGrid_.height };
}

HeightMap::HeightMap(Accumulator &&a
                     , const vr::ReferenceFrame &referenceFrame
                     , double dtmExtractionRadius)
    : referenceFrame_(referenceFrame)
    , tileSize_(a.tileSize_)
    , tileGrid_(tileSize_.width - 1, tileSize_.height - 1)
    , lod_(a.lod_)
    , tileRange_(a.tileRange_)
    , sizeInTiles_(a.sizeInTiles_)
    , sizeInPixels_(calculateSizeInPixels(sizeInTiles_))
    , pane_(sizeInPixels_.height, sizeInPixels_.width, CV_32F
            , cv::Scalar(def::InvalidHeight))
    , worldExtents_(worldExtents(lod_, tileRange_, referenceFrame_))
{
    LOG(info2) << "Copying heightmaps from tiles into one pane.";
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

    debugDump(*this, "hm-plain.png");
    dtmize(pane_, std::ceil(dtmExtractionRadius));
    debugDump(*this,"hm-dtmized.png");
}

namespace {

class HeightMapRaster
    : public imgproc::BoundsValidator<HeightMapRaster>
{
public:
    typedef cv::Vec<float, 1> value_type;
    typedef float channel_type;

    explicit HeightMapRaster(const cv::Mat &mat, float invalidValue)
        : mat_(mat), invalidValue_(invalidValue)
    {}

    int channels() const { return 1; };

    const value_type& operator()(int x, int y) const {
        return mat_.at<value_type>(y, x);
    }

    channel_type saturate(double value) const {
        return value;
    }

    channel_type undefined() const { return invalidValue_; }

    int width() const { return mat_.cols; }
    int height() const { return mat_.rows; }
    math::Size2i size() const { return { mat_.cols, mat_.rows }; }

    bool valid(int x, int y) const {
        return (imgproc::BoundsValidator<HeightMapRaster>::valid(x, y)
                && (mat_.at<float>(y, x) != invalidValue_));
    }

private:
    const cv::Mat &mat_;
    float invalidValue_;
};

} // namespace

void HeightMap::resize(vts::Lod lod)
{
    if (lod > lod_) {
        LOGTHROW(err2, std::runtime_error)
            << "Heightmap can be only shrinked.";
    }
    // no-op if same lod
    if (lod == lod_) { return; }

    LOG(info2) << "Resizing heightmap from LOD " << lod_ << " to LOD "
               << lod << ".";

    // calculate new tile range
    // go up in the tile tree
    vts::TileId ll(lod_, tileRange_.ll(0), tileRange_.ll(1));
    vts::TileId ur(lod_, tileRange_.ur(0), tileRange_.ur(1));
    auto localId(local(lod, ll));
    int scale(1 << localId.lod);
    ll = parent(ll, localId.lod);
    ur = parent(ur, localId.lod);

    vts::TileRange tileRange(ll.x, ll.y, ur.x, ur.y);
    math::Size2 sizeInTiles(math::size(tileRange));
    ++sizeInTiles.width; ++sizeInTiles.height;
    math::Size2 sizeInPixels(calculateSizeInPixels(sizeInTiles));
    math::Point2i offset(-localId.x * tileGrid_.width
                         , -localId.y * tileGrid_.height);

    // create new pane
    cv::Mat tmp(sizeInPixels.height, sizeInPixels.width, CV_32F
                , cv::Scalar(def::InvalidHeight));

    // filter heightmap from pane_ into tmp using filter
    HeightMapRaster srcRaster(pane_, def::InvalidHeight);
    math::CatmullRom2 filter(4.0 * localId.lod, 4.0 * localId.lod);
    for (int j(0); j < tmp.rows; ++j) {
        for (int i(0); i < tmp.cols; ++i) {
            // map x,y into original pane (applying scale and tile offset)
            math::Point2 srcPos(scale * i + offset(0)
                                , scale * j + offset(1));

            if (srcRaster.valid(srcPos(0), srcPos(1))) {
                auto nval(imgproc::reconstruct(srcRaster, filter, srcPos)[0]);
                tmp.at<float>(j, i) = nval;
            }
        }
    }

    // set new content
    lod_ = lod;
    tileRange_ = tileRange;
    sizeInTiles_ = sizeInTiles;
    sizeInPixels_ = sizeInPixels;
    std::swap(tmp, pane_);
    worldExtents_ = worldExtents(lod_, tileRange_, referenceFrame_);

    debugDump(*this, str(boost::format("hm-%d.png") % lod_));
}

vts::NavTile::pointer HeightMap::navtile(const vts::TileId &tileId) const
{
    if (tileId.lod != lod_) {
        LOGTHROW(err2, std::runtime_error)
            << "Cannot generate navtile for " << tileId << " from data at LOD "
            << lod_ << ".";
    }

    // find place for tile data and copy if in valid range
    if (!inside(tileRange_, point(tileId))) {
        LOG(info1) << "No navtile data for tile " << tileId << ".";
        return {};
    }

    // create navtile
    auto nt(std::make_shared<vts::opencv::NavTile>());

    // we know that tileId is inside tileRange, get pixel offset
    math::Point2i offset((tileId.x - tileRange_.ll(0)) * tileGrid_.width
                         , (tileId.y - tileRange_.ll(1)) * tileGrid_.height);

    // get tile data
    cv::Mat tile(pane_, cv::Range(offset(1), offset(1) + tileSize_.height)
                 , cv::Range(offset(0), offset(0) + tileSize_.width));

    // creat mask

    // copy data from pane
    nt->data(tile);

    // optimistic approach: start with full mask, unset invalid nodes
    auto &cm(nt->coverageMask());
    for (auto j(0); j < tile.rows; ++j) {
        for (auto i(0); i < tile.cols; ++i) {
            if (tile.at<float>(j, i) == def::InvalidHeight) {
                cm.set(i, j, false);
            }
        }
    }

    // done
    return nt;
}

void HeightMap::dump(const boost::filesystem::path &filename) const
{
    float min(def::Infinity);
    float max(-def::Infinity);

    for (auto j(0); j < pane_.rows; ++j) {
        for (auto i(0); i < pane_.cols; ++i) {
            auto value(pane_.at<float>(j, i));
            if (value == def::InvalidHeight) { continue; }
            min = std::min(min, value);
            max = std::max(max, value);
        }
    }

    cv::Mat image(pane_.rows, pane_.cols, CV_8UC3, cv::Scalar(255, 0, 0));

    auto size(max - min);
    if (size >= 1e-6) {
        for (auto j(0); j < pane_.rows; ++j) {
            for (auto i(0); i < pane_.cols; ++i) {
                auto value(pane_.at<float>(j, i));
                if (value == def::InvalidHeight) { continue; }
                auto v(std::uint8_t(std::round((255 * (value - min)) / size)));
                image.at<cv::Vec3b>(j, i) = cv::Vec3b(v, v, v);
            }
        }
    }

    create_directories(filename.parent_path());
    imwrite(filename.string(), image);
}

HeightMap::BestPosition HeightMap::bestPosition() const
{
    // TODO: what to do if centroid is not at valid pixel?

    BestPosition out;
    auto &c(out.location);

    math::Extents2i validExtents(math::InvalidExtents{});

    long count(0);
    for (int j(0); j < pane_.rows; ++j) {
        for (int i(0); i < pane_.cols; ++i) {
            if (pane_.at<float>(j, i) == def::InvalidHeight) { continue; }

            c(0) += i;
            c(1) += j;
            ++count;
            update(validExtents, math::Point2i(i, j));
        }
    }

    if (count) {
        c(0) /= count;
        c(1) /= count;
    } else {
        auto cc(math::center(validExtents));
        c(0) = cc(0);
        c(1) = cc(1);
    }

    // sample height
    c(2) = pane_.at<float>(c(1), c(0));

    auto es(math::size(worldExtents_));
    auto eul(ul(worldExtents_));

    auto worldX([&](double x) {
            return eul(0) + ((x * es.width) / sizeInPixels_.width);
        });
    auto worldY([&](double y) {
            return eul(1) - ((y * es.height) / sizeInPixels_.height);
        });

    auto world([&](const math::Point3 &p) {
            return math::Point3(worldX(p(0)), worldY(p(1)), p(2));
        });

    {
        math::Point2 d1(c - validExtents.ll);
        math::Point2 d2(validExtents.ur - c);
        double distance(std::max({ d1(0), d1(1), d2(0), d2(1) }));

        // vertical extent points
        math::Point3 e1(c(0), c(1) - distance, c(2));
        math::Point3 e2(c(0), c(1) + distance, c(2));

        e1 = world(e1);
        e2 = world(e2);

        distance = boost::numeric::ublas::norm_2(e2 - e1);

        out.verticalExtent = distance;
    }

    c = world(c);

    // done
    return out;
}
