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

#include <limits>

#include <boost/filesystem.hpp>
#include <boost/format.hpp>
#include <boost/range/adaptor/reversed.hpp>

#include <opencv2/highgui/highgui.hpp>

#include "math/transform.hpp"

#include "imgproc/rastermask/cvmat.hpp"
#include "imgproc/scanconversion.hpp"
#include "imgproc/const-raster.hpp"
#include "imgproc/svg.hpp"

#include "../../opencv/colors.hpp"
#include "./support.hpp"
#include "./coverage.hpp"

// debug
#include "../../opencv/navtile.hpp"

namespace fs = boost::filesystem;

namespace vtslibs { namespace vts { namespace merge {

namespace {

namespace svg {

template <typename Color>
void rasterize(std::ostream &os, const MeshOpInput &input
               , const Color &color, const TileId &diff
               , const math::Size2 &size = Mesh::coverageSize())
{
    os << "<!-- mask: id=" << input.id() << ", name="
       << input.name() << " -->\n";

    // size of source pixel in destination pixels
    int pixelSize(1 << diff.lod);

    // offset in destination pixels
    cv::Point2i offset(diff.x * size.width, diff.y * size.height);

    LOG(info1) << "Rasterize coverage " << input.tileId()
               << " (diff: " << diff
               << "): pixel size: " << pixelSize
               << ", offset " << offset
               << (input.watertight() ? " (watertight)" : "")
               << ".";

    cv::Rect bounds(0, 0, size.width, size.height);

    auto draw([&](uint xstart, uint ystart, uint size, bool)
    {
        // scale
        xstart *= pixelSize;
        ystart *= pixelSize;
        size *= pixelSize;

        // shift
        xstart -= offset.x;
        ystart -= offset.y;

        // construct rectangle and intersect it with bounds
        cv::Rect r(xstart, ystart, size, size);
        auto rr(r & bounds);

        os << "<rect x=\"" << rr.x << "\" y=\"" << rr.y
           << "\" width=\"" << rr.width << "\" height=\"" << rr.height
           << "\" style=\"" << imgproc::svg::fill(color)
           << ";stroke:none;stroke-width:0\" />\n";
    });

    if (input.watertight()) {
        // fully covered -> simulate full coverage
        draw(0, 0, size.width, true);
        return;
    }

    input.mesh().coverageMask.forEachNode
        (draw, Mesh::CoverageMask::Filter::white);
}

template <typename Color>
void rasterize(std::ostream &os, const imgproc::Contour &contour
               , const Color &color)
{
    const auto size(contour.border.dims());

    auto draw([&](uint xstart, uint ystart, uint size)
    {
        os << "<rect x=\"" << xstart << "\" y=\"" << ystart
           << "\" width=\"" << size << "\" height=\"" << size
           << "\" style=\"" << imgproc::svg::fill(color)
           << ";stroke:none;stroke-width:0\" />\n";
    });

    for (int j(0); j < size.height; ++j) {
        for (int i(0); i < size.width; ++i) {
            if (contour.border.get(i, j)) {
                    draw(i, j, 1);
              }
        }
    }
}

} // namespace svg

void rasterize(const MeshOpInput &input, const cv::Scalar &color
               , cv::Mat &coverage
               , const TileId &diff = TileId())
{
    // size of source pixel in destination pixels
    int pixelSize(1 << diff.lod);

    // offset in destination pixels
    cv::Point2i offset(diff.x * coverage.cols, diff.y * coverage.rows);

    LOG(info1) << "Rasterize coverage " << input.tileId()
               << " (diff: " << diff
               << "): pixel size: " << pixelSize
               << ", offset " << offset
               << (input.watertight() ? " (watertight)" : "")
               << ".";

    cv::Rect bounds(0, 0, coverage.cols, coverage.rows);

    auto draw([&](uint xstart, uint ystart, uint size, bool)
    {
        // scale
        xstart *= pixelSize;
        ystart *= pixelSize;
        size *= pixelSize;

        // shift
        xstart -= offset.x;
        ystart -= offset.y;

        // construct rectangle and intersect it with bounds
        cv::Rect r(xstart, ystart, size, size);
        auto rr(r & bounds);
        cv::rectangle(coverage, rr, color, CV_FILLED, 4);
    });

    if (input.watertight()) {
        // fully covered -> simulate full coverage
        auto s(Mesh::coverageSize());
        draw(0, 0, s.width, true);
        return;
    }

    input.mesh().coverageMask.forEachNode
        (draw, Mesh::CoverageMask::Filter::white);
}

void rasterize(const MeshOpInput &input, const cv::Scalar &color
               , cv::Mat &coverage
               , const NodeInfo &srcNodeInfo
               , const NodeInfo &dstNodeInfo)
{
    (void) input;
    (void) color;
    (void) coverage;
    (void) srcNodeInfo;
    (void) dstNodeInfo;

    LOG(warn3)
        << "Cross SRS fallback merge is unsupported so far. "
        << "Skipping fallback tile.";
}

void storeHeight(Coverage::HeightMap &hm, int x, int y, float z)
{
    auto &value(hm(y, x));
    if (z < value[0]) { value[0] = z; }
    if (z > value[1]) { value[1] = z; }
}

const auto InvalidMin(std::numeric_limits<float>::infinity());
const auto InvalidMax(-std::numeric_limits<float>::infinity());

} // namespace

Coverage::Coverage(const TileId &tileId, const NodeInfo &nodeInfo
                   , const Input::list &sources
                   , bool needCookieCutters)
    : tileId(tileId), sources(sources), hasHoles(false)
    , indices(sources.back().id() + 1, false)
    , cookieCutters(sources.back().id() + 1)
{
    generateCoverage(nodeInfo);
    analyze();
    if (needCookieCutters) { findCookieCutters(); }
}

void Coverage::getSources(Output &output, const Input::list &navtileSource)
    const
{
    for (const auto &input : sources) {
        if (indices[input.id()]) {
            output.source.mesh.push_back(input);
        }
    }
    output.source.navtile
            = filterSources(output.source.mesh, navtileSource);
}

boost::tribool Coverage::covered(const Face &face
                                 , const math::Points3d &vertices
                                 , Input::Id id)
{
    bool hit(false);
    bool miss(false);

    // TODO: repeat border for some distance to take triangles from the tile
    // margin into account
    std::vector<imgproc::Scanline> scanlines;

    const math::Point3 *tri[3] = {
        &vertices[face[0]]
        , &vertices[face[1]]
        , &vertices[face[2]]
    };

    imgproc::scanConvertTriangle
        (*tri[0], *tri[1], *tri[2], 0, coverage.rows, scanlines);

    for (const auto &sl : scanlines) {
        imgproc::processScanline(sl, 0, coverage.cols
                                 , [&](int x, int y, float z)
        {
            // triangle passes through
            if (coverage(y, x) == id) {
                hit = true;
                storeHeight(hm, x, y, z);
            } else {
                miss = true;
            }
        });

        // early exit partially covered face
        if (hit && miss) { return boost::indeterminate; }
    }

    // do one more check in case the triangle is thinner than one pixel
    for (int i = 0; i < 3; ++i) {
        int x(std::round((*tri[i])(0)));
        int y(std::round((*tri[i])(1)));

        if ((x < 0) || (x >= coverage.cols)) {
            continue;
        }
        if ((y < 0) || (y >= coverage.rows)) {
            continue;
        }

        if (coverage(y, x) == id) {
            hit = true;

            storeHeight(hm, x, y, (*tri[i])(2));
        } else {
            miss = true;
        }
    }

    // both hits and misses -> partially covered face
    if (hit && miss) { return boost::indeterminate; }
    return hit;
}

Coverage::Hit Coverage::hit(const Face &face, const math::Points3d &vertices
                            , Input::Id id) const
{
    Hit hit = { false, false };

    // TODO: repeat border for some distance to take triangles from the tile
    // margin into account
    std::vector<imgproc::Scanline> scanlines;

    const math::Point3 *tri[3] = {
        &vertices[face[0]]
        , &vertices[face[1]]
        , &vertices[face[2]]
    };

    imgproc::scanConvertTriangle
        (*tri[0], *tri[1], *tri[2], 0, coverage.rows, scanlines);

    for (const auto &sl : scanlines) {
        imgproc::processScanline(sl, 0, coverage.cols
                                 , [&](int x, int y, float)
        {
            // triangle passes through
            hit.inside = true;
            hit.covered |= (coverage(y, x) == id);
        });

        // early exit partially covered face
        if (hit.covered) { return hit; }
    }

    // do one more check in case the triangle is thinner than one pixel
    for (int i = 0; i < 3; ++i) {
        int x(std::round((*tri[i])(0)));
        int y(std::round((*tri[i])(1)));

        if ((x < 0) || (x >= coverage.cols)) { continue; }
        if ((y < 0) || (y >= coverage.rows)) { continue; }

        hit.inside |= true;
        if (coverage(y, x) == id) {
            hit.covered = true;
            return hit;
        }
    }

    // no hit at all
    return hit;
}

void Coverage::generateCoverage(const NodeInfo &nodeInfo)
{
    // prepare coverage map
    auto coverageSize(Mesh::coverageSize());
    // set coverage to invalid index
    coverage.create(coverageSize.height, coverageSize.width);
    coverage = pixel_type(-1);
    // set heightmap to infinity
    hm.create(coverageSize.height, coverageSize.width);
    hm = HeightMapValue(InvalidMin, InvalidMax);

    // BEGIN OPTIMIZATION {

    // analyze input and skip all sets masked by first watertight tile
    auto iinput([&]() -> Input::list::const_iterator
    {
        for (auto rinput(sources.rbegin()), reinput(sources.rend());
             rinput != reinput; ++rinput)
        {
            if (rinput->watertight()) {
                return std::prev(rinput.base());
            }
        }

        // not found -> just from start
        return sources.begin();
    }());

    // single source set to true if we are left with just one tile
    if ((std::distance(iinput, sources.end()) == 1)
            && iinput->watertight())
    {
        // one source tile and it is watertight -> fully covered by this
        // tile
        static_cast<cv::Mat&>(coverage) = cv::Scalar(iinput->id());

        // single sourced and marked in indices
        indices[*(single = iinput->id())] = true;

        // tile is watertight -> there are no holes at all

        // done
        return;
    }

    // } END OPTIMIZATION

    // process all sources from (limited) bottom to the top
    for (auto einput(sources.end()); iinput != einput; ++iinput) {
        const auto &input(*iinput);

        if (nodeInfo.srs() == input.nodeInfo().srs()) {
            // same SRS -> mask is rendered as is (possible scale and shift)
            rasterize(input, input.id(), coverage
                      , local(input.tileId().lod, tileId));
        } else {
            rasterize(input, input.id(), coverage
                      , input.nodeInfo(), nodeInfo);
        }
    }
}

void Coverage::analyze() {
    // BEGIN OPTIMIZATION {
    if (single) {
        // already analyzed during coverage generation
        topmost_ = *single;
        return;
    }
    // } END OPTIMIZATION

    for (auto j(0); j < coverage.rows; ++j) {
        for (auto i(0); i < coverage.cols; ++i) {
            auto v(coverage(j, i));
            if (v == -1) {
                hasHoles = true;
            } else {
                indices[v] = true;
            }
        }
    }

    // special handling of top-level mesh with empty mask
    if (sources.back().mesh().coverageMask.empty()) {
        // top level tile exists but has empty mask (i.e. probably empty
        // mesh): we have to include it to glue generation otherwise no glue
        // will exist and original (empty) tile will be rendered
        indices[sources.back().id()] = true;
        hasHoles = true;
    }

    // count valid source, remember last one (i.e. topmost one)
    int count(0);
    for (Input::Id id(0), eid(indices.size()); id != eid; ++id) {
        if (!indices[id]) { continue; }
        topmost_ = id;
        ++count;
    }

    // single dataset: topmost -> single
    if (count == 1) { single = topmost_; }
}

void Coverage::findCookieCutters() {
    // single case -> no mesh clipping needed, do not find any cookie cutter
    if (single) { return; }

    // prepare workplace
    cv::Mat binary
        (coverage.rows + 2, coverage.cols + 2, CV_8UC1, cv::Scalar(0));
    cv::Mat view(binary, cv::Range(1, coverage.rows + 1)
                 , cv::Range(1, coverage.cols + 1));

    // find cookie cutter, in reverse order (from topmost surface)
    imgproc::FindContour findContour;
    for (const auto &source : boost::adaptors::reverse(sources)) {
        const auto id(source.id());

        // skip invalid surfaces
        if (!indices[id]) { continue; }

        const auto raster
            (imgproc::cvConstRaster<Coverage::pixel_type>(coverage));
        typedef decltype(raster) RasterType;

        // find contours and place to proper place
        auto &cookieCutter(cookieCutters[id]);
        cookieCutter
            = findContour(raster, [id](const RasterType::value_type &value)
                          {
                              return (value(0) == id);
                          });

        LOG(info1) << "Found " << cookieCutter.rings.size()
                   << " cookie cutters for source: id=" << id << ", name="
                   << source.name() << ".";
    }
}

void Coverage::dump(const fs::path &dump) const
{
    const auto &colors(opencv::palette256vec);

    cv::Mat_<cv::Vec3b> img(coverage.rows, coverage.cols);

    std::transform(coverage.begin(), coverage.end(), img.begin()
                   , [&](pixel_type v) { return colors[v + 1]; });

    const auto filename
        (dump / str(boost::format("coverage-%s.png") % tileId));
    create_directories(filename.parent_path());
    imwrite(filename.string(), img);
}

void Coverage::dumpCookieCutters(std::ostream &os) const
{
    const auto size(Mesh::coverageSize());

    imgproc::svg::Svg svgDoc
        (os, { ((size.width + 2) * 4), ((size.height + 2) * 4) });

    imgproc::svg::Tag scaleGroup
        (os, "g", [](std::ostream &os)
         { os << "transform=\"scale(4)\""; });

    os << "<rect width=\"" << (size.width + 2)
       << "\" height=\"" << (size.height + 2)
       << "\" style=\"fill:black;stroke:none,stroke-width:0\" />\n";

    // shift by 1 pixel to start of tile
    imgproc::svg::Tag tileShiftGroup
        (os, "g", [](std::ostream &os)
         { os << "transform=\"translate(1, 1)\""; });

    for (const auto &input : sources) {
        const auto id(input.id());
        if (!indices[id]) { continue; }

        auto color(opencv::palette256vec[id + 1]);
        color[0] *= .5; color[1] *= .5; color[2] *= .5;

        svg::rasterize(os, input, imgproc::svg::color(color)
                       , local(input.tileId().lod, tileId)
                       , size);
    };

    os << "\n";

    for (const auto &input : sources) {
        const auto id(input.id());
        if (!indices[id]) { continue; }

        const auto &contour(cookieCutters[id]);

        auto color(opencv::palette256vec[id + 1]);
        color[0] *= .7; color[1] *= .7; color[2] *= .7;

        os << "<!-- border: id=" << id << ", name="
           << input.name() << " -->\n";

        svg::rasterize(os, contour, imgproc::svg::color(color, 0.5));
    };

    os << "\n";

    // shift to grid coordinates
    imgproc::svg::Tag px2gridGroup
        (os, "g", [](std::ostream &os)
         { os << "transform=\"translate(0.5, 0.5)\""; });

    for (const auto &input : sources) {
        const auto id(input.id());
        if (!indices[id]) { continue; }

        const auto &color(opencv::palette256vec[id + 1]);
        os << "<!-- contour: id=" << input.id() << ", name="
           << input.name() << " -->\n";

        const auto &contour(cookieCutters[id]);

        for (const auto &ring : contour.rings) {
            if (ring.empty()) { continue; }

            os << "<polygon points=\"";
            for (const auto &point : ring) {
                os << point(0) << ',' << point(1) << ' ';
            }

            os << "\" style=\"fill:none;"
               << imgproc::svg::stroke(color, 0.8)
               << ";stroke-width:1\""
               << " vector-effect=\"non-scaling-stroke\""
               << " />\n";
        }
    }
}

void Coverage::dumpCookieCutters(const fs::path &dumpDir) const
{
    const auto filename(str(boost::format("cookie-cutters-%s.svg")
                            % tileId));
    fs::create_directories(dumpDir);
    const auto path(dumpDir / filename);

    std::ofstream f;
    f.exceptions(std::ios::badbit | std::ios::failbit);
    f.open(path.string(), std::ios_base::out | std::ios_base::trunc);
    dumpCookieCutters(f);
    f.close();
}

boost::optional<double> Coverage::hmMin(int x, int y) const
{
    if ((x < 0) || (y < 0) || (x >= hm.cols) || (y >= hm.rows)) {
        return boost::none;
    }

    const auto value(hm(y, x)[0]);
    if (value != InvalidMin) { return value; }
    return boost::none;
}

boost::optional<double> Coverage::hmMax(int x, int y) const
{
    if ((x < 0) || (y < 0) || (x >= hm.cols) || (y >= hm.rows)) {
        return boost::none;
    }

    const auto value(hm(y, x)[1]);
    if (value != InvalidMax) { return value; }
    return boost::none;
}

boost::optional<double> Coverage::hmAvg(int x, int y) const
{
    if ((x < 0) || (y < 0) || (x >= hm.cols) || (y >= hm.rows)) {
        return boost::none;
    }

    const auto &value(hm(y, x));

    if (value[0] != InvalidMin) {
        if (value[1] != InvalidMax) {
            return (value[0] + value[1]) / 2.0;
        }
        return value[0];
    }

    if (value[1] != InvalidMax) { return value[1]; }

    return boost::none;
}

void Coverage::dilateHm()
{
    HeightMap tmp(hm.rows, hm.cols);

    const std::size_t total(hm.rows * hm.cols);

    auto filter([&]() -> void
    {

        for (std::size_t idx(0); idx < total; ++idx) {
            int x(idx % hm.cols);
            int y(idx / hm.cols);

            HeightMapValue newValue(InvalidMin, InvalidMax);

            for (int i = -1; i <= 1; ++i) {
                const int xx(x + i);
                if ((xx < 0) || (xx >= hm.cols)) { continue; }

                const auto &value(hm(y, xx));
                if (value[0] < newValue[0]) { newValue[0] = value[0]; }
                if (value[1] > newValue[1]) { newValue[1] = value[1]; }
            }

            // store
            tmp(y, x) = newValue;
        }
    });
    filter();
    cv::transpose(tmp, hm);
    filter();
    cv::transpose(tmp, hm);

#if 0
    auto save([&](int index, const cv::Mat &data) -> void
    {
       const auto filename
           (str(boost::format("hm-%s-%s.jpg") % tileId % index));

       std::ofstream f;
       f.exceptions(std::ios::badbit | std::ios::failbit);
       f.open(filename, std::ios_base::out | std::ios_base::trunc);

       opencv::NavTile nt(data);

       int invalid(0);

       auto &cm(nt.coverageMask());
       for (auto j(0); j < data.rows; ++j) {
           for (auto i(0); i < data.cols; ++i) {
               if (!std::isfinite(data.at<float>(j, i))) {
                   cm.set(i, j, false);
                   ++invalid;
               }
           }
       }

       nt.serialize(f);

       f.close();
    });

    std::vector<cv::Mat> channels;
    cv::split(hm, channels);
    save(0, channels[0]);
    save(1, channels[1]);
#endif
}

} } } // namespace vtslibs::vts::merge
