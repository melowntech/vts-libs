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

#include <boost/filesystem.hpp>
#include <boost/format.hpp>
#include <boost/range/adaptor/reversed.hpp>

#include <opencv2/highgui/highgui.hpp>

#include "math/transform.hpp"

#include "imgproc/rastermask/cvmat.hpp"
#include "imgproc/scanconversion.hpp"
#include "imgproc/const-raster.hpp"

#include "../../opencv/colors.hpp"
#include "./support.hpp"
#include "./coverage.hpp"

namespace fs = boost::filesystem;

namespace vtslibs { namespace vts { namespace merge {

namespace {

struct SvgColor { const cv::Vec3b &value; };

template<typename CharT, typename Traits>
inline std::basic_ostream<CharT, Traits>&
operator<<(std::basic_ostream<CharT, Traits> &os, const SvgColor &color)
{
    return os << "rgb(" << (unsigned int)(color.value[0])
              << "," << (unsigned int)(color.value[1])
              << "," << (unsigned int)(color.value[2])
              << ")";
}

void rasterizeSvg(std::ostream &os, const MeshOpInput &input
                  , const cv::Vec3b &color, const TileId &diff
                  , const math::Size2 &size = Mesh::coverageSize())
{
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
           << "\" style=\"fill:" << SvgColor{color}
           << ";stroke:none;stroke-width:0\" />\n";
    });

    if (input.watertight()) {
        // fully covered -> simulate full coverage
        draw(0, 0, size.width, true);
        return;
    }

    input.mesh().coverageMask.forEachNode
        (draw, Mesh::CoverageMask::Filter::white);

    (void) os;
}

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

} // namespace

Coverage::Coverage(const TileId &tileId, const NodeInfo &nodeInfo
                   , const Input::list &sources)
    : tileId(tileId), sources(sources), hasHoles(false)
    , indices(sources.back().id() + 1, false)
    , cookieCutters(sources.back().id() + 1)
{
    generateCoverage(nodeInfo);
    analyze();
    findCookieCutters();
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
                                 , Input::Id id) const
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
                                 , [&](int x, int y, float)
        {
            // triangle passes through
            if (coverage(y, x) == id) {
                hit = true;
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
        } else {
            miss = true;
        }
    }

    // both hits and misses -> partially covered face
    if (hit && miss) { return boost::indeterminate; }
    return hit;
}

void Coverage::generateCoverage(const NodeInfo &nodeInfo)
{
    // prepare coverage map
    auto coverageSize(Mesh::coverageSize());
    coverage.create(coverageSize.height, coverageSize.width);

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

    // set coverage to invalid index
    static_cast<cv::Mat&>(coverage) = cv::Scalar(-1);

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
    imgproc::FindContours findContours;
    for (const auto &source : boost::adaptors::reverse(sources)) {
        const auto id(source.id());

        // skip invalid surfaces, skip the topmost surfaces
        if (!indices[id] || topmost(id)) { continue; }

        const auto raster
            (imgproc::cvConstRaster<Coverage::pixel_type>(coverage));
        typedef decltype(raster) RasterType;

        // find contours and place to proper place
        cookieCutters[id]
             = findContours(raster, [id](const RasterType::value_type &value)
                            {
                                return (value(0) == id);
                            });

        LOG(info1) << "Found " << cookieCutters.back().size()
                   << " cookie cutters for source: "
                   << source.name() << ".";
    }

    // reverse cookie cutter to match list of surfaces
    std::reverse(cookieCutters.begin(), cookieCutters.end());
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

void Coverage::dumpCookieCutters(const fs::path &dumpDir) const
{
    const auto size(Mesh::coverageSize());

    const auto filename(str(boost::format("cookie-cutters-%s.svg")
                            % tileId));
    fs::create_directories(dumpDir);
    const auto path(dumpDir / filename);

    std::ofstream f;
    f.exceptions(std::ios::badbit | std::ios::failbit);
    f.open(path.string(), std::ios_base::out | std::ios_base::trunc);

    f << "<?xml version=\"1.0\"?>\n"
      << "<svg xmlns=\"http://www.w3.org/2000/svg\"\n"
      << "     xmlns:xlink=\"http://www.w3.org/1999/xlink\"\n"
      << "     width=\"" << ((size.width + 2) * 4)
      << "\" height=\"" << ((size.height + 2) * 4)
      << "\">\n";

    f << "<g transform=\"scale(4)\">\n";
    f << "<rect width=\"" << (size.width + 2)
      << "\" height=\"" << (size.height + 2)
      << "\" style=\"fill:black;stroke:none,stroke-width:0\" />\n";

    f << "<g transform=\"translate(1, 1)\">\n";
    for (const auto &input : sources) {
        if (!indices[input.id()]) { continue; }
        auto color(opencv::palette256vec[input.id() + 1]);
        color[0] *= .5; color[1] *= .5; color[2] *= .5;
        rasterizeSvg(f, input, color
                     , local(input.tileId().lod, tileId)
                     , size);
    };

    f << "\n";

    f << "<g transform=\"translate(0.5, 0.5)\">\n";
    auto iinput(sources.begin());
    for (const auto contours : cookieCutters) {
        const auto &input(*iinput++);
        const auto &color(opencv::palette256vec[input.id() + 1]);
        for (const auto &contour : contours) {
            if (contour.empty()) { continue; }

            f << "<polygon points=\"";
            for (const auto &point : contour) {
                f << point(0) << ',' << point(1) << ' ';
            }
            f << "\" style=\"fill:none;stroke:" << SvgColor{color}
              << ";stroke-width:1\""
              << " vector-effect=\"non-scaling-stroke\""
              << " />\n";
        }
    }
    f << "</g>\n";

    f << "</g>\n";
    f << "</g>\n";
    f << "</svg>\n";

    f.close();
}

} } } // namespace vtslibs::vts::merge
