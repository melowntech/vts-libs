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

#include <cmath>
#include <queue>

#include <boost/filesystem.hpp>
#include <boost/format.hpp>

#include <opencv2/highgui/highgui.hpp>

#include "dbglog/dbglog.hpp"

#include "utility/format.hpp"
#include "utility/openmp.hpp"

#include "math/transform.hpp"

#include "imgproc/filtering.hpp"
#include "imgproc/reconstruct.hpp"
#include "imgproc/transformation.hpp"

#include "geo/geodataset.hpp"

#include "io.hpp"
#include "tileop.hpp"
#include "csconvertor.hpp"
#include "opencv/navtile.hpp"

#include "heightmap.hpp"

namespace vtslibs { namespace vts {

constexpr HeightMapBase::DataType HeightMapBase::Infinity;
constexpr HeightMapBase::DataType HeightMapBase::InvalidHeight;

namespace def {
const auto *DumpDir(::getenv("HEIGHTMAP_DUMP_DIR"));
} // namespace def

namespace {

class HeightMapRaster
    : public imgproc::BoundsValidator<HeightMapRaster>
{
public:
    typedef HeightMap::DataType channel_type;
    typedef cv::Vec<channel_type, 1> value_type;

    explicit HeightMapRaster(const cv::Mat &mat, channel_type invalidValue)
        : mat_(mat), invalidValue_(invalidValue), invalidPixel_(invalidValue)
    {}

    int channels() const { return 1; };

    const value_type& operator()(int x, int y) const {
        return mat_.at<value_type>(y, x);
    }

    channel_type saturate(double value) const {
        return value;
    }

    value_type undefined() const { return invalidPixel_; }

    int width() const { return mat_.cols; }
    int height() const { return mat_.rows; }
    math::Size2i size() const { return { mat_.cols, mat_.rows }; }

    bool valid(int x, int y) const {
        return (imgproc::BoundsValidator<HeightMapRaster>::valid(x, y)
                && (mat_.at<channel_type>(y, x) != invalidValue_));
    }

private:
    const cv::Mat &mat_;
    channel_type invalidValue_;
    value_type invalidPixel_;
};

} // namespace

HeightMap::Accumulator::Accumulator(vts::Lod lod)
    : lod_(lod)
    , tileSize_(vts::NavTile::size())
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

    // create new tile (all values set to +oo
    auto res
        (tiles_.insert
         (Tiles::value_type
          (index, cv::Mat
           (tileSize_.height, tileSize_.width, vts::opencv::NavTile::CvDataType
            , cv::Scalar(InvalidHeight)))));
    update(tileRange_, index);

    return res.first->second;
}

namespace {

template <typename Operator>
class Morphology {
public:
    typedef typename Operator::value_type value_type;

    /** Apply morphology operator to data matrix in given kernel
     * radius. Application uses tmp matrix for temporary result.
     *
     *  Operator is applied on in horizontal direction. To work in vertical
     *  direction set transpose to true.
     *
     *  If invalidValue is set reset output to given value.
     */
    Morphology(cv::Mat &data, cv::Mat &tmp, int kernelRadius
               , bool transpose
               , boost::optional<value_type> invalidValue)
        : in_(data), tmp_(tmp), invalidValue_(invalidValue)
    {
        if (transpose) {
            // transpose and swap
            cv::transpose(in_, tmp_);
            std::swap(in_, tmp_);
        }

        // re-create tmp matrix to match input matrix
        tmp_.create(in_.rows, in_.cols, in_.type());

        if (invalidValue_) {
            // invalidate all pixels
            tmp_ = cv::Scalar(*invalidValue_);
        }

        run(kernelRadius);

        // copy result to input matrix
        if (transpose) {
            // transpose back
            cv::transpose(tmp_, in_);
        } else {
            // just swap
            std::swap(in_, tmp_);
        }
    }

private:
    void run(int kernelRadius);

    bool valid(value_type value) const {
        return !invalidValue_ || (value != *invalidValue_);
    }

    cv::Mat &in_;
    cv::Mat &tmp_;
    const boost::optional<value_type> invalidValue_;
};

template <typename Operator>
void Morphology<Operator>::run(int kernelRadius)
{
    const std::size_t total(in_.rows * in_.cols);

    UTILITY_OMP(parallel for)
    for (std::size_t idx = 0; idx < total; ++idx) {
        int x = idx % in_.cols;
        int y = idx / in_.cols;

        // skip invalid data
        if (!valid(in_.at<value_type>(y, x))) { continue; }

        Operator op;

        for (int i = -kernelRadius; i <= kernelRadius; ++i) {
            const int xx(x + i);
            if ((xx < 0) || (xx >= in_.cols)) { continue; }

            auto value(in_.at<value_type>(y, xx));
            if (!valid(value)) { continue; }
            op(value);
        }

        // store only valid value
        if (!op.valid()) { continue; }

        // store
        tmp_.at<value_type>(y, x) = op.get();
    }
}

template <typename ValueType>
class Erosion {
public:
    typedef ValueType value_type;
    static constexpr value_type InvalidValue
    = std::numeric_limits<value_type>::max();

    Erosion() : value_(InvalidValue) {}

    inline void operator()(value_type value) {
        value_ = std::min(value_, value);
    }

    inline bool valid() const { return value_ != InvalidValue; }
    inline value_type get() const { return value_; }

private:
    value_type value_;
};

template <typename ValueType>
class Dilation {
public:
    typedef ValueType value_type;
    static constexpr value_type InvalidValue
    = std::numeric_limits<value_type>::lowest();

    inline Dilation() : value_(InvalidValue) {}

    inline void operator()(value_type value) {
        value_ = std::max(value_, value);
    }

    inline bool valid() const { return value_ != InvalidValue; }
    inline value_type get() const { return value_; }

private:
    value_type value_;
};

void dtmize(cv::Mat &pane, int count)
{
    LOG(info3) << "Generating DTM from heightmap ("
               << pane.cols << "x" << pane.rows << " pixels).";

    cv::Mat tmp;

    LOG(info2) << "Eroding heightmap Y (radius " << count << "px).";
    Morphology<Erosion<HeightMap::DataType>>
        (pane, tmp, count, true, HeightMap::InvalidHeight);
    LOG(info2) << "Eroding heightmap X (radius " << count << "px).";
    Morphology<Erosion<HeightMap::DataType>>
        (pane, tmp, count, false, HeightMap::InvalidHeight);

    LOG(info2) << "Dilating heightmap Y (radius " << count << "px).";
    Morphology<Dilation<HeightMap::DataType>>
        (pane, tmp, count, true, HeightMap::InvalidHeight);
    LOG(info2) << "Dilating heightmap X (radius " << count << "px).";
    Morphology<Dilation<HeightMap::DataType>>
        (pane, tmp, count, false, HeightMap::InvalidHeight);
}

template <typename ...Args>
void debugDump(const HeightMap &hm, const std::string &format, Args &&...args)
{
    if (!def::DumpDir || hm.empty()) { return; }

    hm.dump(boost::filesystem::path(def::DumpDir)
            / utility::format(format, std::forward<Args>(args)...));
}

math::Extents2 worldExtents(vts::Lod lod, const vts::TileRange &tileRange
                            , const registry::ReferenceFrame &referenceFrame
                            , std::string &srs)
{
    if (!valid(tileRange)) { return math::Extents2(math::InvalidExtents{}); }

    vts::NodeInfo nodeLl
        (referenceFrame
         , vts::TileId(lod, tileRange.ll(0), tileRange.ur(1)));
    vts::NodeInfo nodeUr
        (referenceFrame
         , vts::TileId(lod, tileRange.ur(0), tileRange.ll(1)));

    if (!compatible(nodeLl, nodeUr)) {
        LOGTHROW(err2, std::runtime_error)
            << "Heightmap can be constructed from nodes in the "
            "same node subtree.";
    }
    srs = nodeLl.srs();

    return math::Extents2(nodeLl.extents().ll, nodeUr.extents().ur);
}

math::Size2 calculateSizeInTiles(const vts::TileRange &tileRange)
{
    if (!valid(tileRange)) { return {}; }

    math::Size2 s(math::size(tileRange));
    ++s.width;
    ++s.height;
    return s;
}

math::Size2 calculateSizeInPixels(const math::Size2 &tileGrid
                                  , const math::Size2 &sizeInTiles)
{
    if (empty(sizeInTiles)) { return {}; }
    return { 1 + sizeInTiles.width * tileGrid.width
            , 1 + sizeInTiles.height * tileGrid.height };
}

} // namespace

HeightMapBase::HeightMapBase(const registry::ReferenceFrame &referenceFrame
                             , vts::Lod lod, vts::TileRange tileRange)
    : referenceFrame_(&referenceFrame)
    , tileSize_(vts::NavTile::size())
    , tileGrid_(tileSize_.width - 1, tileSize_.height - 1)
    , lod_(lod)
    , tileRange_(tileRange)
    , sizeInTiles_(calculateSizeInTiles(tileRange_))
    , sizeInPixels_(calculateSizeInPixels(tileGrid_, sizeInTiles_))
    , pane_(sizeInPixels_.height, sizeInPixels_.width
            , vts::opencv::NavTile::CvDataType
            , cv::Scalar(InvalidHeight))
{
    updateWorldExtents(worldExtents(lod_, tileRange_, *referenceFrame_, srs_));
}

void HeightMapBase::updateWorldExtents(const math::Extents2 &we)
{
    worldExtents_ = we;
    world2Grid_ = boost::numeric::ublas::identity_matrix<double>(4);
    const auto es(size(worldExtents_));

    // scales
    math::Size2f scale((pane_.cols - 1) / es.width
                       , (pane_.rows - 1) / es.height);

    // scale to grid
    world2Grid_(0, 0) = scale.width;
    world2Grid_(1, 1) = -scale.height;

    // shift
    world2Grid_(0, 3) = -worldExtents_.ll(0) * scale.width;
    world2Grid_(1, 3) = worldExtents_.ur(1) * scale.height;
}

HeightMap::HeightMap(Accumulator &&a
                     , const registry::ReferenceFrame &referenceFrame
                     , double dtmExtractionRadius)
    : HeightMapBase(referenceFrame, a.lod_, a.tileRange_)
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
    debugDump(*this, "hm-dtmized.png");
}

namespace {

HeightMap::Accumulator::Index index(const vts::TileId &tileId)
{
    return HeightMap::Accumulator::Index(tileId.x, tileId.y);
}

vts::Lod lod(const vts::MeshOpInput::list &source)
{
    return (source.empty() ? 0 : source.front().tileId().lod);
}

vts::TileRange tileRange(const vts::MeshOpInput::list &source)
{
    vts::TileRange tr(math::InvalidExtents{});
    for (const auto &input : source) {
        if (input.hasNavtile()) {
            update(tr, index(input.tileId()));
        }
    }
    return tr;
}

} // namespace

HeightMap::HeightMap(const vts::TileId &tileId
                     , const vts::MeshOpInput::list &source
                     , const registry::ReferenceFrame &referenceFrame)
    : HeightMapBase(referenceFrame, lod(source), tileRange(source))
{
    for (const auto &input : source) {
        if (!input.hasNavtile()) { continue; }

        // copy tile in proper place
        Accumulator::Index offset(index(input.tileId()) - tileRange_.ll);
        offset(0) *= tileGrid_.width;
        offset(1) *= tileGrid_.height;
        cv::Mat tile(pane_, cv::Range(offset(1), offset(1) + tileSize_.height)
                     , cv::Range(offset(0), offset(0) + tileSize_.width));

        const auto &nt(input.navtile());
        nt.data().copyTo(tile, renderCoverage(nt));
    }

    // debug
    debugDump(*this, "src-hm-%s.png", tileId);
}

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
    auto localId(vts::local(lod, ll));
    int scale(1 << localId.lod);
    ll = vts::parent(ll, localId.lod);
    ur = vts::parent(ur, localId.lod);

    vts::TileRange tileRange(ll.x, ll.y, ur.x, ur.y);
    math::Size2 sizeInTiles(math::size(tileRange));
    ++sizeInTiles.width; ++sizeInTiles.height;
    math::Size2 sizeInPixels(calculateSizeInPixels(tileGrid_, sizeInTiles));
    math::Point2i offset(-localId.x * tileGrid_.width
                         , -localId.y * tileGrid_.height);

    // TODO compute scale and offset properly

    // create new pane
    cv::Mat tmp(sizeInPixels.height, sizeInPixels.width
                , vts::opencv::NavTile::CvDataType
                , cv::Scalar(InvalidHeight));

    // filter heightmap from pane_ into tmp using filter
    HeightMapRaster srcRaster(pane_, InvalidHeight);
    math::CatmullRom2 filter(4.0 * localId.lod, 4.0 * localId.lod);
    for (int j(0); j < tmp.rows; ++j) {
        for (int i(0); i < tmp.cols; ++i) {
            // map x,y into original pane (applying scale and tile offset)
            math::Point2 srcPos(scale * i + offset(0)
                                , scale * j + offset(1));

            // reconstruct value
            tmp.at<DataType>(j, i)
                = imgproc::reconstruct(srcRaster, filter, srcPos)[0];
        }
    }

    // set new content
    lod_ = lod;
    tileRange_ = tileRange;
    sizeInTiles_ = sizeInTiles;
    sizeInPixels_ = sizeInPixels;
    std::swap(tmp, pane_);
    updateWorldExtents(worldExtents(lod_, tileRange_, *referenceFrame_, srs_));

    debugDump(*this, "hm-%d.png", lod_);
}

vts::NavTile::pointer HeightMap::navtile(const vts::TileId &tileId) const
{
    if (tileId.lod != lod_) {
        LOGTHROW(err2, std::runtime_error)
            << "Cannot generate navtile for " << tileId << " from data at LOD "
            << lod_ << ".";
    }

    // find place for tile data and copy if in valid range
    if (!inside(tileRange_, vts::point(tileId))) {
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
            if (tile.at<DataType>(j, i) == InvalidHeight) {
                cm.set(i, j, false);
            }
        }
    }

    // done
    return nt;
}

namespace {

double findNearestValid(const cv::Mat &pane, int x, int y)
{
    cv::Mat_<std::uint8_t> seen(pane.rows, pane.cols, std::uint8_t(false));
    std::queue<math::Point2i> queue;

    const auto add([&](int x, int y) -> void
    {
        // was this place seen alreay?
        auto &marker(seen(y, x));
        if (marker) { return; }
        marker = true;

        queue.emplace(x, y);
    });

    int xend(pane.cols - 1);
    int yend(pane.rows - 1);

    // start with initial point
    add(x, y);

    while (!queue.empty()) {
        const auto p(queue.front());
        queue.pop();

        const auto &value(pane.at<HeightMapBase::DataType>(p(1), p(0)));
        if (value != HeightMapBase::InvalidHeight) { return value; }

        if (p(0) > 0) { add(p(0) - 1, p(1)); }
        if (p(0) < xend) { add(p(0) + 1, p(1)); }
        if (p(1) > 0) { add(p(0), p(1) - 1); }
        if (p(1) < yend) { add(p(0), p(1)  + 1); }
    }

    // not found, force zero
    return 0.0;
}

} // namespace

void HeightMap::dump(const boost::filesystem::path &filename) const
{
    DataType min(Infinity);
    DataType max(-Infinity);

    for (auto j(0); j < pane_.rows; ++j) {
        for (auto i(0); i < pane_.cols; ++i) {
            auto value(pane_.at<DataType>(j, i));
            if (value == InvalidHeight) { continue; }
            min = std::min(min, value);
            max = std::max(max, value);
        }
    }

    cv::Mat image(pane_.rows, pane_.cols, CV_8UC3, cv::Scalar(255, 0, 0));

    auto size(max - min);
    if (size >= 1e-6) {
        for (auto j(0); j < pane_.rows; ++j) {
            for (auto i(0); i < pane_.cols; ++i) {
                auto value(pane_.at<DataType>(j, i));
                if (value == InvalidHeight) { continue; }
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

    if (!pane_.cols || !pane_.rows) { return out; }

    auto &c(out.location);

    math::Extents2i validExtents(math::InvalidExtents{});

    long count(0);
    for (int j(0); j < pane_.rows; ++j) {
        for (int i(0); i < pane_.cols; ++i) {
            if (pane_.at<DataType>(j, i) == InvalidHeight) { continue; }

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

    if (count) {
        c(2) = pane_.at<DataType>(c(1), c(0));
    } else {
        // no valid pixel -> zero
        c(2) = 0;
    }

    if (c(2) == InvalidHeight) {
        // invalid pixel samples, find nearest valid pixel
        // TODO: interpolate value

        c(2) = findNearestValid(pane_, c(0), c(1));
    }

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
        math::Point2 d1(math::Point2(c(0), c(1)) - validExtents.ll);
        math::Point2 d2(validExtents.ur - math::Point2(c(0), c(1)));
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

namespace {

math::Extents2 addHalfPixel(const math::Extents2 &e
                            , const math::Size2 gs)
{
    auto es(size(e));
    const math::Size2f pixelSize(es.width / gs.width, es.height / gs.height);
    const math::Point2 halfPixel(pixelSize.width / 2.0
                                 , pixelSize.height / 2.0);
    return math::Extents2(e.ll - halfPixel, e.ur + halfPixel);
}

} // namespace

void HeightMap::warp(const registry::ReferenceFrame &referenceFrame
                     , vts::Lod lod, const vts::TileRange &tileRange)
{
    if (empty()) { return; }

    // TODO: work with native type inside GeoDataset when GeoDataset interface
    // is ready

    math::Size2 sizeInTiles(calculateSizeInTiles(tileRange));
    math::Size2 sizeInPixels(calculateSizeInPixels(tileGrid_, sizeInTiles));

    std::string srs;
    auto extents(worldExtents(lod, tileRange, referenceFrame, srs));

    auto srcSrs(registry::system.srs(srs_));
    auto dstSrs(registry::system.srs(srs));

    // create source dataset (extents are inflate by half pixel in each
    // direction to facilitate grid registry)
    auto srcDs(geo::GeoDataset::create
               ("", srcSrs.srsDef
                , addHalfPixel(worldExtents_, sizeInPixels_)
                , sizeInPixels_
                , geo::GeoDataset::Format::dsm
                (geo::GeoDataset::Format::Storage::memory)
                , InvalidHeight));

    // prepare mask
    {
        auto &cm(srcDs.mask());
        // optimistic approach: start with full mask, unset invalid nodes
        cm.reset(true);
        for (auto j(0); j < pane_.rows; ++j) {
            for (auto i(0); i < pane_.cols; ++i) {
                if (pane_.at<DataType>(j, i) == InvalidHeight) {
                    cm.set(i, j, false);
                }
            }
        }
    }

    // copy data inside dataset
    pane_.convertTo(srcDs.data(), srcDs.data().type());
    srcDs.flush();

    // create destination dataset (extents are inflate by half pixel in each
    // direction to facilitate grid registry)
    auto dstDs(geo::GeoDataset::deriveInMemory
               (srcDs, dstSrs.srsDef, sizeInPixels
                , addHalfPixel(extents, sizeInPixels)));

    // warp
    srcDs.warpInto(dstDs);

    /** fix Z component
     * for each valid point XY in navtile:
     *     X'Y' = conv2d(XY, srs -> referenceFrame_.model.navigationSrs)
     *     Z' = conv3d(X'Y'Z, referenceFrame_.model.navigationSrs
     *                , referenceFrame.model.navigationSrs).Z
     */
    {
        // temporary pane
        cv::Mat tmp(sizeInPixels.height, sizeInPixels.width
                    , vts::opencv::NavTile::CvDataType
                    , cv::Scalar(InvalidHeight));
        // 2d convertor between srs and referenceFrame_.model.navigationSrs
        const vts::CsConvertor sds2srcNav
            (srs, referenceFrame_->model.navigationSrs);

        const vts::CsConvertor srcNav2dstNav
            (referenceFrame_->model.navigationSrs
             , referenceFrame.model.navigationSrs);

        const auto &hf(dstDs.cdata());
        dstDs.cmask().forEach([&](int x, int y, bool)
        {
            // compose 2D point in srs
            const math::Point2 sdsXY
                (dstDs.raster2geo(math::Point2(x, y), 0.0));
            // height from current heightmap
            const auto srcZ(hf.at<double>(y, x));

            // convert to source nav system
            const auto mavXY(sds2srcNav(sdsXY));

            // nox, compose 3D point in source nav system and convert to
            // destination nav system and store Z component into destination
            // heightmap
            const auto dstZ
                (srcNav2dstNav(math::Point3(mavXY(0), mavXY(1), srcZ))(2));
            tmp.at<float>(y, x) = dstZ;
        }, geo::GeoDataset::Mask::Filter::white);

        // done, swap panes
        std::swap(tmp, pane_);
    }

    // set new content
    lod_ = lod;
    tileRange_ = tileRange;
    sizeInTiles_ = sizeInTiles_;
    sizeInPixels_ = sizeInPixels_;
    srs_ = srs;
    updateWorldExtents(extents);
    referenceFrame_ = &referenceFrame;
}

void HeightMap::warp(const vts::NodeInfo &nodeInfo)
{
    warp(nodeInfo.referenceFrame(), nodeInfo.nodeId().lod
         , vts::TileRange(point(nodeInfo)));
    debugDump(*this, "hm-warped-%s-%s.png", srs_, nodeInfo.nodeId());
}

// inlines

boost::optional<HeightMap::DataType>
HeightMap::reconstruct(const math::Point2 &point) const
{
    // sanity check
    if (!math::inside(worldExtents_, point)) { return boost::none; }

    // try to reconstruct
    const HeightMapRaster raster(pane_, InvalidHeight);
    const auto height(imgproc::reconstruct
                      (raster, math::CatmullRom2(2.0, 2.0)
                       , math::transform(world2Grid_, point))[0]);

    if (height == InvalidHeight) {
        return boost::none;
    }

    return height;
}

void HeightMap::halve()
{
    // compute new dimension, limit size to at least 2 grid samples (i.e. 1
    // pixel)
    sizeInPixels_.width = std::round(sizeInPixels_.width / 2);
    sizeInPixels_.height = std::round(sizeInPixels_.height / 2);

    if (sizeInPixels_.width < 2) { sizeInPixels_.width = 2; }
    if (sizeInPixels_.height < 2) { sizeInPixels_.height = 2; }

    // create new pane
    cv::Mat tmp(sizeInPixels_.height, sizeInPixels_.width
                , vts::opencv::NavTile::CvDataType
                , cv::Scalar(InvalidHeight));

    // compute scaling from destination raster to source raster
    imgproc::GridScaling2 scaling(math::Size2(tmp.cols, tmp.rows)
                                  , math::Size2(pane_.cols, pane_.rows));
    const auto der(scaling.derivatives({}));

    // filter heightmap from pane_ into tmp using filter
    HeightMapRaster srcRaster(pane_, InvalidHeight);
    math::CatmullRom2 filter(2.0 * std::max(der(0), 1.0)
                             , 2.0 * std::max(der(1), 1.0));

    for (int j(0); j < tmp.rows; ++j) {
        for (int i(0); i < tmp.cols; ++i) {
            // map x,y into original pane via scaling mapper
            const auto srcPos(scaling.map(math::Point2(i, j)));

            // reconstruct value
            tmp.at<DataType>(j, i)
                = imgproc::reconstruct(srcRaster, filter, srcPos)[0];
        }
    }

    // set new content, keep other stuff intact
    std::swap(tmp, pane_);

    // force world2grid matrix recomputations
    updateWorldExtents(worldExtents_);

    // dump halving information
    debugDump(*this, "halve-%s.png", sizeInPixels_);
}

void dtmize(geo::GeoDataset &dataset, const math::Size2 &count)
{
    // get double matrix from dataset
    auto &pane(dataset.data());

    LOG(info3) << "Generating DTM from heightmap ("
               << pane.cols << "x" << pane.rows << " pixels).";

    cv::Mat tmp;

    auto ndv(dataset.rawNodataValue());

    LOG(info2) << "Eroding heightmap Y (radius " << count.height << "px).";
    Morphology<Erosion<double>> (pane, tmp, count.height, true, ndv);
    LOG(info2) << "Eroding heightmap X (radius" << count.width << "px).";
    Morphology<Erosion<double>>(pane, tmp, count.width, false, ndv);

    LOG(info2) << "Dilating heightmap Y (radius " << count.height << "px).";
    Morphology<Dilation<double>>(pane, tmp, count.height, true, ndv);
    LOG(info2) << "Dilating heightmap X (radius " << count.width << " px).";
    Morphology<Dilation<double>>(pane, tmp, count.width, false, ndv);
}

} } // vtslibs::vts

