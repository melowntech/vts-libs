#include <array>

#include <boost/format.hpp>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

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

    std::string utf8() const {
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

private:
    static const char* utf8(std::uint8_t v) {
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
};

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

class Frontier {
public:
    Frontier() = default;
    Frontier(const TileId &origin, long tileSize)
        : origin_(origin), tileSize_(tileSize)
    {}

    operator bool() const { return !tiles_.empty(); }

    void add(long x, long y, Vicinity vicinity) {
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

    void debug(const math::Size2 &size) const;

private:
    struct LocalTile {
        long easting;
        long northing;

        /** presence flags of all 8 neighbours
         */
        Vicinity vicinity;

        typedef std::vector<LocalTile> list;

        LocalTile(long easting, long northing, Vicinity vicinity)
            : easting(easting), northing(northing), vicinity(vicinity)
        {}
    };

    LocalTile::list tiles_;
    TileId origin_;
    long tileSize_;
};

Frontier::Tile::list Frontier::tiles()
{
    // sort tiles
    std::sort(tiles_.begin(), tiles_.end()
              , [](const LocalTile &l, const LocalTile &r) -> bool
    {
        if (l.northing < r.northing) { return true; }
        else if (r.northing < l.northing) { return false; }
        return l.easting < r.easting;
    });

    const LocalTile *prev(nullptr);
    Tile::list tiles;
    for (const auto &tile : tiles_) {
        if (prev && (prev->easting == tile.easting)
           && (prev->northing == tile.northing))
        {
            // same position as previous tile, merge vicinity
            tiles.back().vicinity.merge(tile.vicinity);
            continue;
        }

        // new tile
        tiles.emplace_back
            (TileId(origin_.lod, origin_.easting + tile.easting * tileSize_
                    , origin_.northing + tile.northing * tileSize_)
             , tile.vicinity);
        prev = &tile;
    }
    return tiles;
}

void Frontier::debug(const math::Size2 &size) const
{
    cv::Mat flags(size.height, size.width, CV_8UC1);
    for (const auto &tile : tiles_) {
        // only for debug (to allow local mask dump)
        auto &current(flags.at<unsigned char>(tile.northing, tile.easting));
        int value(current + 0x40);
        current = (value > 0xff) ? 0xff : value;
    }
    dumpMat(origin_.lod, "flags", flags);
}

void createMask(Frontier &frontier, Lod lod, const RasterMask &rmask
                , int hwin, int margin, const Extents &roi
                , const Point2l &origin, int idx)
{
    const auto size(math::size(roi));
    const auto &offset(roi.ll);

    LOG(info1)
        << "Creating mask at LOD " << lod << ", roi: " << roi
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

#ifdef HM_FILTER_DUMP_FLAGS
    for (int j(0); j < size.height; ++j) {
        for (int i(0); i < size.width; ++i) {
            // only for debug (to allow local mask dump)
            inside.at<unsigned char>(j, i)
                = 0xff * (inside.at<unsigned char>(j, i)
                          && outside.at<unsigned char>(j, i));
        }
    }
    dumpMat(lod, str(boost::format("mask%d") % idx), inside);
#endif
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

Frontier::Tile::list createFrontier(const Properties &properties
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

    LOG(info1) << "Frontier roi in tiles from alignment: " << roi;

    const auto tSize(tileSize(properties, lod));
    Frontier frontier({ lod, properties.alignment(0) + tSize * roi.ll(0)
                        , properties.alignment(1) + tSize * roi.ll(1) }
                      , tSize);

    int i(0);
    auto ilocalRois(localRois.begin());
    auto irois(rois.begin());
    for (const auto *ti : tis) {
        if (const auto *mask = ti->mask(lod)) {
            createMask(frontier, lod, *mask, hwin, margin, *ilocalRois
                       , irois->ll - roi.ll, i);
        }
        ++ilocalRois;
        ++irois;
        ++i;
    }

#ifdef HM_FILTER_DUMP_FLAGS
    frontier.debug(size(roi));
#endif
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

    Window(TileSet::Detail &ts, Lod lod, int margin, const Filter &filter)
        : step_(TileMetadata::HMSize - 1)
        , ts_(ts), tileSize_(tileSize(ts.properties, lod))
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
                   << ftile.vicinity.utf8() << ".";

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

        auto applyDistance(calculateDistance(ftile.vicinity));
        (void) applyDistance;

        for (int j(0); j < TileMetadata::HMSize; ++j) {
            for (int i(0); i < TileMetadata::HMSize; ++i) {
                const auto old(tile->heightmap[j][i]);
                hm[j][i] = (imgproc::reconstruct
                            (*this, filter_, math::Point2(i, j))[0]);
                LOG(info1)
                    << old << " -> " << hm[j][i]
                    << ": diff (" << i << ", " << j << "): "
                    << (old - hm[j][i]);
            }
        }
    }

    bool calculateDistance(const Vicinity &vicinity);

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
        LOG(info4) << "flag: " << std::hex << int(flag);

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
        << vicinity.negate().utf8() << ": m: "
        << m << ", distance: " << distance_;

    return true;
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
void filterHeightmapLod(TileSet::Detail &ts, Lod lod
                        , const TileIndices &update
                        , const Filter &filter)
{
    LOG(info2) << "Filtering height map at LOD " << lod << ".";

    // half-window as an integer; add small epsilon to deal with numeric errors
    auto hwin(filter.halfwinx());
    auto intHwin(int(std::ceil(hwin - 1e-5)));
    auto margin(intHwin);

    // build frontier
    auto frontier(createFrontier(ts.properties, lod, update, intHwin, margin));
    if (frontier.empty()) { return; }

    Window<Filter> window(ts, lod, intHwin, filter);

    // process frontier
    for (const auto &tile : frontier) {
        LOG(info1) << "Filtering tile " << tile.id << ".";
        window.process(tile);
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
