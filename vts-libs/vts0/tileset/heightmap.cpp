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

#include <array>

#include <boost/format.hpp>
#include <boost/filesystem.hpp>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "imgproc/filtering.hpp"
#include "imgproc/fillrect.hpp"

#include "../tileset-detail.hpp"

namespace vtslibs { namespace vts0 {

namespace {

const char *HM_FILTER_DUMP_ROOT("HM_FILTER_DUMP_ROOT");
const char* getDumpDir() {
    const char *dir(std::getenv(HM_FILTER_DUMP_ROOT));
    if (dir) {
        boost::filesystem::create_directories(dir);
    }
    return dir;
}

void dumpMat(const char *dumpDir, Lod lod, const std::string &name
             , const cv::Mat &mat, int idx = -1)
{
    if (!dumpDir) { return; }

    auto file((idx >= 0)
              ? str(boost::format("%s/%02d-%s-%02d.png")
                    % dumpDir % lod % name % idx)
              : str(boost::format("%s/%02d-%s.png")
                    % dumpDir % lod % name));
    cv::Mat tmp;
    flip(mat, tmp, 0);
    cv::imwrite(file, tmp);
}

struct Vicinity {
    std::uint8_t value;
    Vicinity(std::uint8_t value = 0x0) : value(value) {}
    void merge(const Vicinity &o) { value |= o.value; }
    bool full() const { return value == 0xff; }
    Vicinity negate() const { return { std::uint8_t(~value) }; }

    struct Neighbour {
        int x;
        int y;
        std::uint8_t flag;
    };

    typedef std::array<Neighbour, 8> Neighbours;
    static const Neighbours neighbours;

    std::string utf8() const;

private:
    static const char* utf8(std::uint8_t v);
};

std::string Vicinity::utf8() const
{
    std::string s("[");
    for (std::uint8_t i(0), flag(0x80); i < 8; ++i, flag >>= 1) {
        if (value & flag) {
            s.append(utf8(value & flag));
        } else {
            s.push_back(' ');
        }
    }
    s.push_back(']');
    return s;
}

const char* Vicinity::utf8(std::uint8_t v)
{
    switch (v) {
    case 0x01: return "\xe2\x86\x92";
    case 0x02: return "\xe2\x86\x97";
    case 0x04: return "\xe2\x86\x91";
    case 0x08: return "\xe2\x86\x96";
    case 0x10: return "\xe2\x86\x90";
    case 0x20: return "\xe2\x86\x99";
    case 0x40: return "\xe2\x86\x93";
    case 0x80: return "\xe2\x86\x98";
    }
    return "?";
}

const Vicinity::Neighbours Vicinity::neighbours = {{
    { 1, 0, (1 << 0) }
    , { 1, 1, (1 << 1) }
    , { 0, 1, (1 << 2) }
    , { -1, 1, (1 << 3) }
    , { -1, 0, (1 << 4) }
    , { -1, -1, (1 << 5) }
    , { 0, -1, (1 << 6) }
    , { 1, -1, (1 << 7) }
}};

template<typename CharT, typename Traits>
inline std::basic_ostream<CharT, Traits>&
operator<<(std::basic_ostream<CharT, Traits> &os, const Vicinity &v)
{
    return os << v.utf8();
}

class Frontier {
public:
    Frontier() = default;
    Frontier(const TileId &origin)
        : origin_(origin)
    {}

    operator bool() const { return !tiles_.empty(); }

    void add(long x, long y, Vicinity vicinity) {
        LOG(debug) << "adding(" << x << ", " << y << "): " << vicinity << ".";
        tiles_.emplace_back(x, y, vicinity);
    }

    struct Tile {
        TileId id;

        /** presence flags of all 8 neighbours
         */
        Vicinity vicinity;

        typedef std::vector<Tile> list;

        Tile() = default;
        Tile(const TileId &id, Vicinity vicinity)
            : id(id), vicinity(vicinity)
        {}
    };

    Tile::list tiles();

    void debug(const char *dumpDir, const Size2l &size) const;

private:
    struct LocalTile {
        long x;
        long y;

        /** presence flags of all 8 neighbours
         */
        Vicinity vicinity;

        typedef std::vector<LocalTile> list;

        LocalTile(long x, long y, Vicinity vicinity)
            : x(x), y(y), vicinity(vicinity)
        {}
    };

    LocalTile::list tiles_;
    TileId origin_;
};

Frontier::Tile::list Frontier::tiles()
{
    // sort tiles
    std::sort(tiles_.begin(), tiles_.end()
              , [](const LocalTile &l, const LocalTile &r) -> bool
    {
        if (l.y < r.y) { return true; }
        else if (r.y < l.y) { return false; }
        return l.x < r.x;
    });

    const LocalTile *prev(nullptr);
    Tile::list tiles;
    for (const auto &tile : tiles_) {
        if (prev && (prev->x == tile.x)
           && (prev->y == tile.y))
        {
            // same position as previous tile, merge vicinity
            tiles.back().vicinity.merge(tile.vicinity);
            continue;
        }

        // new tile
        tiles.emplace_back
            (TileId(origin_.lod, origin_.x + tile.x
                    , origin_.y + tile.y)
             , tile.vicinity);
        prev = &tile;
    }
    return tiles;
}

void Frontier::debug(const char *dumpDir, const Size2l &size) const
{
    if (!dumpDir) { return; }

    LOG(info1) << "debug: size: " << size;

    cv::Mat flags(size.height, size.width, CV_8UC1);
    flags = cv::Scalar(0x00);
    for (const auto &tile : tiles_) {
        auto &current(flags.at<unsigned char>(tile.y, tile.x));
        int value(current + 0x40);
        current = (value > 0xff) ? 0xff : value;
    }
    dumpMat(dumpDir, origin_.lod, "flags", flags);
}

void createMask(const char *dumpDir, Frontier &frontier
                , Lod lod, const RasterMask &continuous
                , const RasterMask *discrete
                , int hwin, int margin, const Extents &roi
                , const Point2l &origin, int idx)
{
    const auto size(math::size(roi));
    const auto &offset(roi.ll);

    LOG(info1)
        << "Creating mask at LOD " << lod << ", roi: " << roi
        << ", size: " << size << ", origin: " << origin << ".";

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
            imgproc::fillRectangle(outside, rect, cv::Scalar(0x00));
        }
    }

    // render dilated valid quads from continuous mask in inside mask
    // render dilated invalid quads from continuous mask in outside mask
    const auto white(cv::Scalar(0xff));
    continuous.forEachQuad([&](uint xstart, uint ystart, uint xsize
                               , uint ysize, bool valid)
    {
        cv::Point2i start(xstart - offset(0) - hwin
                          , ystart - offset(1) - hwin);
        cv::Point2i end(xstart + xsize - offset(0) + hwin - 1
                        , ystart + ysize - offset(1) + hwin - 1);

        imgproc::fillRectangle((valid ? inside : outside)
                               , start, end, white);
    }, RasterMask::Filter::both);

    // render dilate valid quads from concrete mask in both inside and outside
    // mask
    if (discrete) {
        discrete->forEachQuad([&](uint xstart, uint ystart, uint xsize
                                  , uint ysize, bool)
        {
            cv::Point2i start(xstart - offset(0) - hwin
                              , ystart - offset(1) - hwin);
            cv::Point2i end(xstart + xsize - offset(0) + hwin - 1
                            , ystart + ysize - offset(1) + hwin - 1);

            imgproc::fillRectangle(inside, start, end, white);
            imgproc::fillRectangle(outside, start, end, white);
        }, RasterMask::Filter::white);
    }

    dumpMat(dumpDir, lod, "inside", inside, idx);
    dumpMat(dumpDir, lod, "outside", outside, idx);

    // intersect inside and outside mask, place result into inside mask
    const auto intersect([&](int i, int j) -> bool
    {
        return (inside.at<unsigned char>(j, i)
                && outside.at<unsigned char>(j, i));
    });

    const auto checkedIntersect([&](int i, int j) -> bool
    {
        return ((i >= 0) && (i < size.width)
                && (j >= 0) && (j < size.height)
                && intersect(i, j));
    });

    for (int j(0); j < size.height; ++j) {
        for (int i(0); i < size.width; ++i) {
            if (!intersect(i, j)) { continue; }

            // calculate vicinity flags
            Vicinity vicinity;
            for (const auto &neightbour : Vicinity::neighbours) {
                vicinity.merge
                    (checkedIntersect(i + neightbour.x, j + neightbour.y)
                     * neightbour.flag);
            }
            frontier.add(origin(0) + i, origin(1) + j, vicinity);
        }
    }

    if (dumpDir) {
        for (int j(0); j < size.height; ++j) {
            for (int i(0); i < size.width; ++i) {
                // only for debug (to allow local mask dump)
                inside.at<unsigned char>(j, i)
                    = 0xff * (inside.at<unsigned char>(j, i)
                              && outside.at<unsigned char>(j, i));
            }
        }
        dumpMat(dumpDir, lod, "flags", inside, idx);
    }
}

void renderMasks(cv::Mat &m, const RasterMask *continuous
                 , const RasterMask *discrete, const Point2l &offset)
{
    const cv::Scalar colors[2]{ { 0xff, 0xff, 0xff }, { 0x00, 0x00, 0xff } };
    const cv::Scalar *color(colors);
    for (const auto *mask : { continuous, discrete }) {
        if (mask) {
            mask->forEachQuad([&](uint xstart, uint ystart, uint xsize
                                  , uint ysize, bool)
            {
                cv::Point2i start(xstart - offset(0), ystart - offset(1));
                cv::Point2i end(xstart + xsize - offset(0) - 1
                                , ystart + ysize - offset(1) - 1);
                imgproc::fillRectangle(m, start, end, *color);
            }, RasterMask::Filter::white);
        }
        ++color;
    }
}

/** Calculate area covered by white quads in both rmask and supMask + margin
 *  around.
 *
 *  NB: result.ll() is valid point, result.ur() is first invalid point,
 *  i.e. size(result) is full raster size
 */
Extents coveredArea(const RasterMask *rmask, const RasterMask *supMask
                    , int margin)
{
    Extents extents{math::InvalidExtents()};

    // merge both masks
    for (const auto *mask : { rmask, supMask }) {
        if (!mask) { continue; }

        mask->forEachQuad([&](uint xstart, uint ystart, uint xsize
                              , uint ysize, bool)
        {
            update(extents, Point2l(xstart, ystart));
            update(extents, Point2l(xstart + xsize, ystart + ysize));
        }, RasterMask::Filter::white);
    }

    if (empty(extents)) { return extents; }

    // grow extents by margin
    return extents + margin;
}

Frontier::Tile::list createFrontier(const char *dumpDir
                                    , Lod lod, const TileIndices &continuous
                                    , const TileIndices *discrete
                                    , int hwin, int margin)
{
    std::vector<Extents> localRois;
    std::vector<Extents> rois;
    Extents roi{math::InvalidExtents()};
    for (std::size_t index(0), eindex(continuous.size());
         index != eindex; ++index)
    {
        const auto *ti(continuous[index]);
        // calculate covered area of tiles in tileindex, add margin and hwin
        localRois.push_back
            (coveredArea
             (ti->mask(lod)
              , (discrete ? (*discrete)[index]->mask(lod) : nullptr)
              , margin));

        if (empty(localRois.back())) {
            rois.push_back(localRois.back());
            continue;
        }

        rois.push_back(localRois.back());

        // update common origin
        roi = unite(roi, rois.back());
    }

    if (empty(roi)) { return {}; }

    LOG(info1) << "Frontier roi: " << roi;

    Frontier frontier({ lod, roi.ll(0), roi.ll(1) });

    cv::Mat inputDebug;
    if (dumpDir) {
        inputDebug.create(size(roi).height, size(roi).width, CV_8UC3);
        inputDebug = cv::Scalar(0x00, 0x00, 0x00);
    }

    auto ilocalRois(localRois.begin());
    auto irois(rois.begin());
    for (std::size_t index(0), eindex(continuous.size());
         index != eindex; ++index)
    {
        const auto *ti(continuous[index]);
        if (const auto *mask = ti->mask(lod)) {
            const auto *dmask
                (discrete ? (*discrete)[index]->mask(lod) : nullptr);
            createMask(dumpDir, frontier, lod, *mask, dmask, hwin
                       , margin, *ilocalRois, irois->ll - roi.ll, index);

            if (dumpDir) {
                renderMasks(inputDebug, mask, dmask
                            , (irois->ll - roi.ll + ilocalRois->ll));
            }
        }
        ++ilocalRois;
        ++irois;
    }

    if (dumpDir) {
        frontier.debug(dumpDir, size(roi));
        dumpMat(dumpDir, lod, "input", inputDebug);
    }

    return frontier.tiles();
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

    Window(TileSet::Detail &ts, int margin, const Filter &filter)
        : step_(TileMetadata::HMSize - 1)
        , ts_(ts)
        , margin_(margin), size_(2 * margin_ + 1, 2 * margin_ + 1)
        , raster_(step_ * size_.height + 1
                  , step_ * size_.width + 1, CV_64FC1)
        , mask_(raster_.rows, raster_.cols, CV_8UC1)
        , filter_(filter.scaled(step_, step_))
        , distanceSeed_(TileMetadata::HMSize, TileMetadata::HMSize, CV_8UC1)
        , distance_(TileMetadata::HMSize, TileMetadata::HMSize, CV_32FC1)
    {
        distance_ = cv::Scalar(0);
        LOG(info1)
            << "Using filter with half-window: "
            << filter_.halfwinx() << "x" << filter_.halfwiny();
    }

    // NB: mask and raster are stored upside-down!
    void process(const Frontier::Tile &ftile) {
        const auto &center(ftile.id);
        auto *tile(ts_.findMetaNode(center));
        if (!tile) { return; }

        LOG(info1) << "Processing tile: " << center << " with vicinity "
                   << ftile.vicinity << ".";

        // reset mask
        mask_ = cv::Scalar(0x00);

        TileId id(center.lod);

        // set y to the bottom tile
        id.y = (center.y - margin_);

        // process all tiles
        for (int j(0), y(0); j < size_.height;
             ++j, y += step_, id.y += 1)
        {
            // set x to the left tile
            id.x = (center.x - margin_);
            for (int i(0), x(0); i < size_.width;
                 ++i, x += step_, id.x += 1)
            {
                const auto *node(ts_.findMetaNode(id));
                if (!node) { continue; }
                LOG(info1) << "Loading tile: " << id << " at position ("
                           << i << ", " << j << ").";

                // copy data from tile's heightmap
                for (int jj(0); jj < TileMetadata::HMSize; ++jj) {
                    for (int ii(0); ii < TileMetadata::HMSize; ++ii) {
                        int xx(x + ii), yy(y + jj);
                        auto value(node->heightmap
                                   [TileMetadata::HMSize - 1 - jj][ii]);
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
                imgproc::fillRectangle(mask_, area, cv::Scalar(0xff));
            }
        }

        updated_.emplace_back(tile);
        auto &hm(updated_.back().heightmap);

        auto applyDistance(calculateDistance(ftile.vicinity));

        for (int j(0); j < TileMetadata::HMSize; ++j) {
            for (int i(0); i < TileMetadata::HMSize; ++i) {
                const auto old
                    (tile->heightmap[TileMetadata::HMSize - 1 - j][i]);
                hm[TileMetadata::HMSize - 1 - j][i] = blend
                    (applyDistance, i, j, old
                     , imgproc::reconstruct
                     (*this, filter_, math::Point2(i, j))[0]);
                LOG(info1)
                    << old << " -> " << hm[j][i]
                    << ": diff (" << i << ", " << j << "): "
                    << (old - hm[j][i]);
            }
        }
    }

    double blend(bool applyDistance, int i, int j, double oldValue
                 , double newValue);

    bool calculateDistance(const Vicinity &vicinity);

    void flush() {
        for (auto &node : updated_) { node.flush(); }
    }

    // reconstruction interface follows
    int channels() const { return 1; };
    channel_type saturate(double value) const { return value; }
    value_type undefined() const { return {}; }
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
    const int margin_;
    const math::Size2 size_;
    cv::Mat raster_;
    cv::Mat mask_;

    const Filter filter_;

    Node::list updated_;

    cv::Mat distanceSeed_;
    cv::Mat distance_;
};

template <typename Filter>
bool Window<Filter>::calculateDistance(const Vicinity &vicinity)
{
    if (vicinity.full()) { return false; }

    auto &m(distanceSeed_);
    m = cv::Scalar(0xff);
    const cv::Scalar zero(0x00);
    for (std::uint8_t i(0), flag(1); i < 8; ++i, flag <<= 1) {
        if (vicinity.value & flag) { continue; }

        switch (flag) {
        case 0x01: // right
            m(cv::Rect(m.cols - 1, 0, 1, m.rows)) = zero; break;

        case 0x02: // right/up
            m.at<unsigned char>(m.rows - 1, m.cols - 1) = 0; break;

        case 0x04: // up
            m(cv::Rect(0, m.rows - 1, m.cols, 1)) = zero; break;

        case 0x08: // left/up
            m.at<unsigned char>(m.rows - 1, 0) = 0; break;

        case 0x10: // left
            m(cv::Rect(0, 0, 1, m.rows)) = zero; break;

        case 0x20: // left, down
            m.at<unsigned char>(0, 0) = 0; break;

        case 0x40: // down
            m(cv::Rect(0, 0, m.cols, 1)) = zero; break;

        case 0x80: // down/right
            m.at<unsigned char>(0, m.cols - 1) = 0; break;
        }
    }

    cv::distanceTransform(m, distance_, CV_DIST_C, 3);

    LOG(info1)
        << "Calculated distance map; empty tiles in vicinity: "
        << vicinity.negate() << ": m: "
        << m << ", distance: " << distance_;

    return true;
}

template <typename Filter>
double Window<Filter>::blend(bool applyDistance, int i, int j, double oldValue
                             , double newValue)
{
    // just new value if not blending by distance
    if (!applyDistance) { return newValue; }

    const double max(distance_.rows);
    const auto weight(distance_.at<float>(j, i));

    // blend new and old value
    return (weight * newValue + (max - weight) * oldValue) / max;
}

LodRange lodSpan(const TileIndices &tis)
{
    auto out(LodRange::emptyRange());
    for (const auto *ti : tis) {
        out = unite(out, ti->lodRange());
    }
    return out;
}

template <typename Filter>
void filterHeightmapLod(const char *dumpDir, TileSet::Detail &ts, Lod lod
                        , const TileIndices &continuous
                        , const TileIndices *discrete
                        , const Filter &filter)
{
    LOG(info2) << "Filtering height map at LOD " << lod << ".";

    // half-window as an integer; add small epsilon to deal with numeric errors
    auto hwin(filter.halfwinx());
    auto intHwin(int(std::ceil(hwin - 1e-5)));
    auto margin(intHwin);

    // build frontier
    auto frontier(createFrontier(dumpDir, lod, continuous
                                 , discrete, intHwin, margin));
    if (frontier.empty()) { return; }

    Window<Filter> window(ts, intHwin, filter);

    // process frontier
    for (const auto &tile : frontier) {
        LOG(info1) << "Filtering tile " << tile.id << ".";
        window.process(tile);
    }

    window.flush();
}

} // namespace

void TileSet::Detail::filterHeightmap(const TileIndices &continuous
                                      , const TileIndices *discrete
                                      , double cutoff)
{
    if (continuous.empty()) { return; }

    // tile-level filter
    const math::CatmullRom2 filter(cutoff, cutoff);

    auto lodRange(lodSpan(continuous));
    const char *dumpDir(getDumpDir());
    LOG(info2)
        << "Filtering height map in LOD range " << lodRange << ".";
    for (auto lod : lodRange) {
        filterHeightmapLod(dumpDir, *this, lod, continuous, discrete, filter);
    }
}

} } // namespace vtslibs::vts0
