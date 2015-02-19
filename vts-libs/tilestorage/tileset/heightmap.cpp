#include <boost/format.hpp>

#include <opencv2/highgui/highgui.hpp>

#include "imgproc/filtering.hpp"

#include "../tileset-detail.hpp"

// #define DUMP_FLAGS 1
#undef DUMP_FLAGS

namespace vadstena { namespace tilestorage {

void TileSet::Detail::filterHeightmap(const TileIndex &changed, double hwin)
{
    LOG(info2)
        << "Filtering height map in LOD range "
        << changed.lodRange() << ".";
    for (auto lod : changed.lodRange()) {
        filterHeightmap(lod, changed, hwin);
    }
}

namespace {

void dumpMat(Lod lod, const std::string &name, const cv::Mat &mat)
{
#ifdef DUMP_FLAGS
    auto file(str(boost::format("changed-%s-%02d.png") % name % lod));
    cv::Mat tmp;
    flip(mat, tmp, 0);
    cv::imwrite(file, tmp);
#else
    (void) lod; (void) name; (void) mat;
#endif
}

cv::Mat createMask(Lod lod, const TileIndex &changed, int hwin, int margin)
{
    const auto &rmask(*changed.mask(lod));
    auto size(rmask.dims());
    size.width += (2 * margin);
    size.height += (2 * margin);

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
        cv::Point2i start(xstart + margin - hwin
                          , ystart + margin - hwin);
        cv::Point2i end(xstart + xsize + margin + hwin - 1
                        , ystart + ysize + margin + hwin - 1);

        cv::rectangle((valid ? inside : outside)
                      , start, end, white, CV_FILLED, 4);
    }, RasterMask::Filter::both);

    dumpMat(lod, "inside", inside);
    dumpMat(lod, "outside", outside);

    // intersect inside and outside mask, place result into inside mask
    for (int j(0); j < size.height; ++j) {
        for (int i(0); i < size.width; ++i) {
            inside.at<unsigned char>(j, i)
                = 0xff * (inside.at<unsigned char>(j, i)
                          && outside.at<unsigned char>(j, i));
        }
    }

    dumpMat(lod, "mask", inside);
    return inside;
}

} // namespace

void TileSet::Detail::filterHeightmap(Lod lod, const TileIndex &changed
                                      , double hwin)
{
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

    class Window
        : public imgproc::BoundsValidator<Window>
    {
    public:
        typedef double channel_type;
        typedef cv::Vec<channel_type, 1> value_type;

        Window(TileSet::Detail &ts, Lod lod, double hwin, int margin)
            : step_(TileMetadata::HMSize - 1)
            , ts_(ts), tileSize_(tileSize(ts.properties, lod))
            , margin_(margin), size_(2 * margin_ + 1, 2 * margin_ + 1)
            , raster_(step_ * size_.height + 1
                      , step_ * size_.width + 1, CV_64FC1)
            , mask_(raster_.rows, raster_.cols, CV_8UC1)
            , filter_(step_ * hwin, step_ * hwin)
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
            return (BoundsValidatorType::valid(x, y)
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

        const math::CatmullRom2 filter_;

        Node::list updated_;
    };

    LOG(info2) << "Filtering height map at LOD " << lod << ".";

    // half-window as an integer; add small epsilon to deal with numeric errors
    auto intHwin(int(std::ceil(hwin - 1e-5)));
    auto margin(intHwin);

    const auto getTileId([&](int i, int j)
    {
        return changed.tileId(lod, i - margin, j - margin);
    });

    auto flags(createMask(lod, changed, intHwin, margin));

    Window window(*this, lod, hwin, intHwin);

    // process raster
    for (int j(0); j < flags.rows; ++j) {
        for (int i(0); i < flags.cols; ++i) {
            // get flags
            if (!flags.at<unsigned char>(j, i)) { continue; }

            // regenerate tile
            auto tileId(getTileId(i, j));
            LOG(info2) << "Filtering tile [" << i << ", " << j << "] "
                       << tileId << ".";

            window.process(tileId);
        }
    }

    window.flush();
}

} } // namespace vadstena::tilestorage
