#include <boost/format.hpp>

#include <opencv2/highgui/highgui.hpp>

#include "imgproc/filtering.hpp"

#include "../tileset-detail.hpp"

//#define HM_FILTER_DUMP_FLAGS 1

namespace vadstena { namespace tilestorage {

namespace {

void dumpMat(Lod lod, const std::string &name, const cv::Mat &mat)
{
#ifdef HM_FILTER_DUMP_FLAGS
    auto file(str(boost::format("%02d-changed-%s.png") % lod % name));
    cv::Mat tmp;
    flip(mat, tmp, 0);
    cv::imwrite(file, tmp);
#else
    (void) lod; (void) name; (void) mat;
#endif
}

void createMask(cv::Mat &flags, Lod lod, const RasterMask &rmask
                , int hwin, int margin, const Extents &roi
                , const Point2l &origin, int idx)
{
    const auto size(math::size(roi));
    const auto &offset(roi.ll);

    LOG(info1)
        << "Creating mask at lod" << lod << ", roi: " << roi
        << ", size: " << size;

    // inside mask, clear
    cv::Mat inside(size.height, size.width, CV_8UC1);
    inside = cv::Scalar(0x00);

    // outside mask, fill with dilated outer border
    cv::Mat outside(size.height, size.width, CV_8UC1);
    // 1) full white
    outside = cv::Scalar(0xff);
    // 2) and clear inside
    {
        cv::Rect rect(cv::Point2i(margin + hwin, margin + hwin)
                      , cv::Size(size.width - 2 * (margin + hwin)
                                 , size.height - 2 * (margin + hwin)));
        if ((rect.width > 0) && (rect.height > 0)) {
            cv::rectangle(outside, rect
                          , cv::Scalar(0x00), CV_FILLED, 4);
        }
    }

    // render dilated valid quads in inside mask
    // render dilated invalid quads in outside mask
    const auto white(cv::Scalar(0xff));
    rmask.forEachQuad([&](uint xstart, uint ystart, uint xsize
                          , uint ysize, bool valid)
    {
        cv::Point2i start(xstart - offset(0) - hwin
                          , ystart - offset(1) - hwin);
        cv::Point2i end(xstart + xsize - offset(0) + hwin - 1
                        , ystart + ysize - offset(1) + hwin - 1);

        cv::rectangle((valid ? inside : outside)
                      , start, end, white, CV_FILLED, 4);
    }, RasterMask::Filter::both);

    dumpMat(lod, str(boost::format("inside%d") % idx), inside);
    dumpMat(lod, str(boost::format("outside%d") % idx), outside);

    // intersect inside and outside mask, place result into inside mask
    for (int j(0); j < size.height; ++j) {
        for (int i(0); i < size.width; ++i) {
            if (inside.at<unsigned char>(j, i)
                && outside.at<unsigned char>(j, i))
            {
                flags.at<unsigned char>(origin(1) + j, origin(0) + i) = 0xff;
            }

#ifdef HM_FILTER_DUMP_FLAGS
            // only for debug (to allow local mask dump)
            inside.at<unsigned char>(j, i)
                = 0xff * (inside.at<unsigned char>(j, i)
                          && outside.at<unsigned char>(j, i));
#endif
        }
    }

    dumpMat(lod, str(boost::format("mask%d") % idx), inside);
}

/** Calculate area covered by white quads in rmask + margin around.
 *  NB: result.ll() is valid point, result.ur() is first invalid point,
 *  i.e. size(result) is full raster size
 */
Extents coveredArea(const RasterMask *rmask, int margin)
{
    Extents extents{math::InvalidExtents()};
    if (!rmask) { return extents; }

    rmask->forEachQuad([&](uint xstart, uint ystart, uint xsize
                           , uint ysize, bool)
    {
        update(extents, Point2l(xstart, ystart));
        update(extents, Point2l(xstart + xsize, ystart + ysize));
    }, RasterMask::Filter::white);

    // grow extents by margin
    return extents + margin;
}

class Flags {
public:
    Flags() = default;

    Flags(const cv::Mat &flags, const TileId &origin, long tileSize)
        : flags_(flags), origin_(origin), tileSize_(tileSize)
    {}

    operator bool() const { return flags_.data; }

    int width() const { return flags_.cols; }
    int height() const { return flags_.rows; }

    bool operator()(int x, int y) const {
        return flags_.at<unsigned char>(y, x);
    }

    TileId tileId(int x, int y) const {
        return { origin_.lod, origin_.easting + x * tileSize_
                , origin_.northing + y * tileSize_ };
    }

private:
    cv::Mat flags_;
    TileId origin_;
    long tileSize_;
};

Flags createFlags(const Properties &properties
                  , Lod lod, const TileIndices &tis
                  , int hwin, int margin)
{
    std::vector<Extents> localRois;
    std::vector<Extents> rois;
    Extents roi{math::InvalidExtents()};
    for (const auto *ti : tis) {
        // calculate covered area of tiles in tileindex, add margin and hwin
        localRois.push_back(coveredArea(ti->mask(lod), margin));
        if (empty(localRois.back())) { continue; }
        rois.emplace_back
            (ti->fromReference(properties.alignment
                               , lod, localRois.back().ll)
             , ti->fromReference(properties.alignment
                                 , lod, localRois.back().ur));

        // update common origin
        roi = unite(roi, rois.back());
    }

    if (empty(roi)) { return {}; }

    LOG(info1) << "Flags roi in tiles from alignment: " << roi;

    cv::Mat flags(size(roi).height, size(roi).width, CV_8UC1);
    flags = cv::Scalar(0x00);

    int i(0);
    auto ilocalRois(localRois.begin());
    auto irois(rois.begin());
    for (const auto *ti : tis) {
        if (const auto *mask = ti->mask(lod)) {
            createMask(flags, lod, *mask, hwin, margin, *ilocalRois
                       , irois->ll - roi.ll, i);
        }
        ++ilocalRois;
        ++irois;
        ++i;
    }

    dumpMat(lod, "flags", flags);

    const auto tSize(tileSize(properties, lod));
    return {
        flags, { lod, properties.alignment(0) + tSize * roi.ll(0)
                , properties.alignment(1) + tSize * roi.ll(1) }
        , tSize
    };
}

class Node {
public:
    Node(TileMetadata *node) : node_(node) {}
    void flush() const {
        std::copy
            (&heightmap[0][0]
             , &heightmap[TileMetadata::HMSize - 1][TileMetadata::HMSize]
             , &node_->heightmap[0][0]);
    }

    TileMetadata::Heightmap heightmap;

    typedef std::vector<Node> list;

private:
    TileMetadata *node_;
};

template <typename Filter>
class Window
    : public imgproc::BoundsValidator<Window<Filter> >
{
public:
    typedef double channel_type;
    typedef cv::Vec<channel_type, 1> value_type;

    Window(TileSet::Detail &ts, Lod lod, int margin, const Filter &filter)
        : step_(TileMetadata::HMSize - 1)
        , ts_(ts), tileSize_(tileSize(ts.properties, lod))
        , margin_(margin), size_(2 * margin_ + 1, 2 * margin_ + 1)
        , raster_(step_ * size_.height + 1
                  , step_ * size_.width + 1, CV_64FC1)
        , mask_(raster_.rows, raster_.cols, CV_8UC1)
        , filter_(filter.scaled(step_, step_))
    {
        LOG(info1)
            << "Using filter with half-window: "
            << filter_.halfwinx() << "x" << filter_.halfwiny();
    }

    // NB: mask and raster are stored upside-down!
    void process(const TileId &center) {
        auto *tile(ts_.findMetaNode(center));
        if (!tile) { return; }

        // reset mask
        mask_ = cv::Scalar(0x00);

        TileId id(center.lod);

        // set northing to the bottom tile
        id.northing = (center.northing - tileSize_ * margin_);

        // process all tiles
        for (int j(0), y(0); j < size_.height;
             ++j, y += step_, id.northing += tileSize_)
        {
            // set easting to the left tile
            id.easting = (center.easting - tileSize_ * margin_);
            for (int i(0), x(0); i < size_.width;
                 ++i, x += step_, id.easting += tileSize_)
            {
                const auto *node(ts_.findMetaNode(id));
                if (!node) { continue; }
                LOG(info1) << "Loading tile: " << id << " at position ("
                           << i << ", " << j << ").";

                // copy data from tile's heightmap
                for (int jj(0); jj < TileMetadata::HMSize; ++jj) {
                    for (int ii(0); ii < TileMetadata::HMSize; ++ii) {
                        int xx(x + ii), yy(y + jj);
                        auto value(node->heightmap[jj][ii]);
                        if (mask_.at<unsigned char>(yy, xx)) {
                            // valid value from other tile, use average
                            value += raster_.at<channel_type>(yy, xx);
                            value /= 2.f;
                        }
                        raster_.at<channel_type>(yy, xx) = value;
                    }
                }

                // update mask
                cv::Rect area
                    (cv::Point2i(x, y), cv::Size(TileMetadata::HMSize
                                                 , TileMetadata::HMSize));
                cv::rectangle
                    (mask_, area, cv::Scalar(0xff), CV_FILLED, 4);
            }
        }

        updated_.emplace_back(tile);
        auto &hm(updated_.back().heightmap);

        for (int j(0); j < TileMetadata::HMSize; ++j) {
            for (int i(0); i < TileMetadata::HMSize; ++i) {
                auto old(tile->heightmap[j][i]);
                hm[j][i] = (imgproc::reconstruct
                            (*this, filter_, math::Point2(i, j))[0]);
                LOG(info1)
                    << old << " -> " << hm[j][i]
                    << ": diff (" << i << ", " << j << "): "
                    << (old - hm[j][i]);
            }
        }
    }

    void flush() {
        for (auto &node : updated_) { node.flush(); }
    }

    // reconstruction interface follows
    int channels() const { return 1; };
    channel_type saturate(double value) const { return value; }
    channel_type undefined() const { return 0.f; }
    int width() const { return step_ * size_.width + 1; }
    int height() const { return step_ * size_.height + 1; }
    math::Size2 size() const { return { width(), height() }; }

    bool valid(int x, int y) const {
        x = translate(x); y = translate(y);
        return (imgproc::BoundsValidator<Window<Filter> >::valid(x, y)
                && mask_.at<unsigned char>(y, x));
    }

    const value_type& operator()(int x, int y) const {
        x = translate(x); y = translate(y);
        return raster_.at<value_type>(y, x);
    }

private:
    int translate(int index) const { return index + step_ * margin_; }

    const int step_;
    TileSet::Detail &ts_;
    const long tileSize_;
    const int margin_;
    const math::Size2 size_;
    cv::Mat raster_;
    cv::Mat mask_;

    const Filter &filter_;

    Node::list updated_;
};

LodRange lodSpan(const TileIndices &tis)
{
    auto out(LodRange::emptyRange());
    for (const auto *ti : tis) {
        out = unite(out, ti->lodRange());
    }
    return out;
}

template <typename Filter>
void filterHeightmapLod(TileSet::Detail &ts, Lod lod
                        , const TileIndices &update
                        , const Filter &filter)
{
    LOG(info2) << "Filtering height map at LOD " << lod << ".";

    // half-window as an integer; add small epsilon to deal with numeric errors
    auto hwin(filter.halfwinx());
    auto intHwin(int(std::ceil(hwin - 1e-5)));
    auto margin(intHwin);

    // build flags
    auto flags(createFlags(ts.properties, lod, update, intHwin, margin));
    if (!flags) { return; }

    Window<Filter> window(ts, lod, intHwin, filter);

    // process raster
    for (int j(0); j < flags.height(); ++j) {
        for (int i(0); i < flags.width(); ++i) {
            // get flags
            if (!flags(i, j)) { continue; }

            // regenerate tile
            auto tileId(flags.tileId(i, j));
            LOG(info1) << "Filtering tile [" << i << ", " << j << "] "
                       << tileId << ".";

            window.process(tileId);
        }
    }

    window.flush();
}

} // namespace

void TileSet::Detail::filterHeightmap(const TileIndices &update
                                      , double cutoff)
{
    if (update.empty()) { return; }

    // tile-level filter
    const math::CatmullRom2 filter(cutoff, cutoff);


    auto lodRange(lodSpan(update));
    LOG(info2)
        << "Filtering height map in LOD range " << lodRange << ".";
    for (auto lod : lodRange) {
        filterHeightmapLod(*this, lod, update, filter);
    }
}

} } // namespace vadstena::tilestorage
