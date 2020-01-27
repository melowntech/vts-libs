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
#include <opencv2/imgproc/imgproc.hpp>

#include "math/transform.hpp"

#include "geometry/polygon.hpp"

#include "imgproc/rastermask/cvmat.hpp"
#include "imgproc/scanconversion.hpp"
#include "imgproc/fillrect.hpp"
#include "imgproc/const-raster.hpp"
#include "imgproc/svg.hpp"

#include "../../opencv/colors.hpp"
#include "support.hpp"
#include "coverage.hpp"

// debug
#include "../../opencv/navtile.hpp"

namespace fs = boost::filesystem;

namespace vtslibs { namespace vts { namespace merge {

namespace {

namespace svg {

template <typename Color>
void rasterize(std::ostream &os, const MeshOpInput &input
               , const Color &color, const int margin, const TileId &diff
               , const math::Size2 &size = Mesh::coverageSize())
{
    os << "<!-- mask: id=" << input.id() << ", name="
       << input.name() << " -->\n";

    // size of source pixel in destination pixels
    int pixelSize(1 << diff.lod);

    // offset in destination pixels
    const auto cs(Mesh::coverageSize());
    const cv::Point2i offset(diff.x * cs.width, diff.y * cs.height);

    LOG(info1) << "Rasterize coverage " << input.tileId()
               << " (diff: " << diff
               << "): pixel size: " << pixelSize
               << ", offset " << offset
               << (input.watertight() ? " (watertight)" : "")
               << ".";

    if (input.watertight()) {
        // fully covered
        os << "<rect width=\"" << size.width << "\" height=\"" << size.height
           << "\" style=\"" << imgproc::svg::fill(color)
           << ";stroke:none;stroke-width:0\" />\n";
        return;
    }

    const cv::Rect bounds(0, 0, size.width, size.height);

    const auto draw([&](uint xstart, uint ystart, uint size, bool)
    {
        // scale
        xstart *= pixelSize;
        ystart *= pixelSize;
        size *= pixelSize;

        // shift
        xstart -= offset.x;
        ystart -= offset.y;

        // move to real data start
        xstart += margin;
        ystart += margin;

        // construct rectangle and intersect it with bounds
        cv::Rect r(xstart, ystart, size, size);
        auto rr(r & bounds);

        // expand at tile border
        if (rr.x == margin) { rr.x = 0; rr.width += margin; }
        if (rr.y == margin) { rr.y = 0; rr.height += margin; }
        if ((rr.x + rr.width + margin) == bounds.width) {
            rr.width += margin;
        }
        if ((rr.y + rr.height + margin) == bounds.height) {
            rr.height += margin;
        }

        os << "<rect x=\"" << rr.x << "\" y=\"" << rr.y
           << "\" width=\"" << rr.width << "\" height=\"" << rr.height
           << "\" />\n";
    });

    os << "<g style=\"" << imgproc::svg::fill(color)
       << ";stroke:none;stroke-width:0\">\n";
    input.mesh().coverageMask.forEachNode
        (draw, Mesh::CoverageMask::Filter::white);
    os << "</g>\n";
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
           << "\" />\n";
    });

    os << "<g style=\"" << imgproc::svg::fill(color)
       << ";stroke:none;stroke-width:0\">\n";
    for (int j(0); j < size.height; ++j) {
        for (int i(0); i < size.width; ++i) {
            if (contour.border.get(i, j)) {
                    draw(i, j, 1);
              }
        }
    }
    os << "</g>\n";
}

} // namespace svg

void rasterize(const MeshOpInput &input, const cv::Scalar &color
               , cv::Mat &coverage, int margin
               , const TileId &diff = TileId())
{
    if (input.watertight()) {
        // fully covered -> full coverage
        coverage = color;
        return;
    }

    // size of source pixel in destination pixels
    const int pixelSize(1 << diff.lod);

    // offset in destination pixels
    const auto cs(Mesh::coverageSize());
    const cv::Point2i offset(diff.x * cs.width, diff.y * cs.height);

    LOG(info1) << "Rasterize coverage " << input.tileId()
               << " (diff: " << diff
               << "): pixel size: " << pixelSize
               << ", offset " << offset
               << (input.watertight() ? " (watertight)" : "")
               << ".";

    const auto draw([&](uint xstart, uint ystart, uint size, bool)
    {
        // scale
        xstart *= pixelSize;
        ystart *= pixelSize;
        size *= pixelSize;

        // shift
        xstart -= offset.x;
        ystart -= offset.y;

        // move to real data start
        xstart += margin;
        ystart += margin;

        // construct rectangle for this node
        cv::Rect r(xstart, ystart, size, size);

        // expand at tile border
        if (r.x == margin) { r.x = 0; r.width += margin; }
        if (r.y == margin) { r.y = 0; r.height += margin; }
        if ((r.x + r.width + margin) == coverage.cols) {
            r.width += margin;
        }
        if ((r.y + r.height + margin) == coverage.rows) {
            r.height += margin;
        }

        imgproc::fillRectangle(coverage, r, color);
    });

    input.mesh().coverageMask.forEachNode
        (draw, Mesh::CoverageMask::Filter::white);
}

void rasterize(const MeshOpInput &input, const cv::Scalar &color
               , cv::Mat &coverage
               , const NodeInfo &srcNodeInfo
               , const NodeInfo &dstNodeInfo, int margin)
{
    (void) input;
    (void) color;
    (void) coverage;
    (void) srcNodeInfo;
    (void) dstNodeInfo;
    (void) margin;

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
                   , const Input::list &sources, const MergeOptions &options
                   , bool needCookieCutters)
    : tileId(tileId), sources(sources), options(options)
    , innerSize(Mesh::coverageSize())
    , coverageSize((innerSize.width + 2 * options.safetyMargin)
                   , (innerSize.height + 2 * options.safetyMargin))
    , coverage(coverageSize.height, coverageSize.width, pixel_type(-1))
    , hasHoles(false)
    , indices(sources.back().id() + 1, false)
    , cookieCutters(sources.back().id() + 1)
    , hm(coverageSize.height, coverageSize.width
         , HeightMapValue(InvalidMin, InvalidMax))
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

/** Round continuous coordinate to close pixel value.
 */
inline int nearestPixel(double c) { return std::floor(c); }

boost::tribool Coverage::covered(const Face &face
                                 , const math::Points3d &vertices
                                 , Input::Id id)
{
    bool hit(false);
    bool miss(false);

    // TODO: repeat border for some distance to take triangles from the tile
    // margin into account

    const math::Point3 *tri[3] = {
        &vertices[face[0]]
        , &vertices[face[1]]
        , &vertices[face[2]]
    };

    std::vector<imgproc::Scanline> scanlines;
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


        // TODO: use early exit only when samples heightmap is not needed
        // // early exit partially covered face
        // if (hit && miss) { return boost::indeterminate; }
    }

    // do one more check in case the triangle is thinner than one pixel
    for (int i = 0; i < 3; ++i) {
        int x(nearestPixel((*tri[i])(0)));
        int y(nearestPixel((*tri[i])(1)));

        if ((x < 0) || (x >= coverage.cols)
            || (y < 0) || (y >= coverage.rows))
        {
            // vertex outside -> mix
            miss = true;
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
        int x(nearestPixel((*tri[i])(0)));
        int y(nearestPixel((*tri[i])(1)));

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
            rasterize(input, input.id(), coverage, options.safetyMargin
                      , local(input.tileId().lod, tileId));
        } else {
            rasterize(input, input.id(), coverage
                      , input.nodeInfo(), nodeInfo, options.safetyMargin);
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

inline imgproc::ChainSimplification
chainSimplification(ContourSimplification simplification)
{
    switch (simplification) {
    case ContourSimplification::none:
        return imgproc::ChainSimplification::none;
    case ContourSimplification::straight:
        return imgproc::ChainSimplification::simple;
    case ContourSimplification::rdp:
        return imgproc::ChainSimplification::rdp;
    }

    // default
    return imgproc::ChainSimplification::rdp;
}

void Coverage::findCookieCutters() {
    // single case -> no mesh clipping needed, do not find any cookie cutter
    if (single) { return; }

    const auto kernel
        (cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3)));

    // NB: cookie cutters are pre-allocated in ctor
    MeshOpInput::Id id(0);
    for (auto valid : indices) {
        if (!valid) { ++id; continue; }

        // make mask from current surface and dilate by one pixel
        MatType mat(coverage == id);
        if (!topmost(id)) {
            cv::dilate(mat, mat, kernel);
        }

        const auto raster
            (imgproc::cvConstRaster<Coverage::pixel_type>(mat));

        typedef decltype(raster)::value_type value_type;

        cookieCutters[id]
            = (imgproc::findContour
               (raster
                , [&](const value_type &value) { return value[0]; }
                , imgproc::ContourParameters(imgproc::PixelOrigin::corner)
                .setSimplification
                (chainSimplification(options.contourSimplification))
                .setRdpMaxError(options.rdpMaxError))
               );

        ++id;
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
    imgproc::svg::Svg svgDoc
        (os, { ((coverageSize.width + 2) * 4)
                , ((coverageSize.height + 2) * 4) });

    imgproc::svg::Tag scaleGroup
        (os, "g", [](std::ostream &os)
         { os << "transform=\"scale(4)\""; });

    os << "<rect width=\"" << (coverageSize.width + 2)
       << "\" height=\"" << (coverageSize.height + 2)
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
                       , options.safetyMargin
                       , local(input.tileId().lod, tileId)
                       , coverageSize);
    };

    os << "\n";

#if 0
    // border information
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
#endif

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
    if (std::isfinite(value)) { return value; }
    return boost::none;
}

boost::optional<double> Coverage::hmMax(int x, int y) const
{
    if ((x < 0) || (y < 0) || (x >= hm.cols) || (y >= hm.rows)) {
        return boost::none;
    }

    const auto value(hm(y, x)[1]);
    if (std::isfinite(value)) { return value; }
    return boost::none;
}

boost::optional<double> Coverage::hmAvg(int x, int y) const
{
    if ((x < 0) || (y < 0) || (x >= hm.cols) || (y >= hm.rows)) {
        return boost::none;
    }

    const auto &value(hm(y, x));

    if (std::isfinite(value[0])) {
        if (std::isfinite(value[1])) {
            return (value[0] + value[1]) / 2.0;
        }
        return value[0];
    }

    if (std::isfinite(value[1])) { return value[1]; }

    return boost::none;
}

void Coverage::dilateHm()
{
    HeightMap tmp(hm.rows, hm.cols);

    const std::size_t total(hm.rows * hm.cols);

    auto ptmp(&tmp);

    auto filter([&]() -> void
    {
        UTILITY_OMP(parallel for)
        for (std::size_t idx = 0; idx < total; ++idx) {
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
            (*ptmp)(y, x) = newValue;
        }
    });
    filter();
    cv::transpose(tmp, hm);
    filter();
    cv::transpose(tmp, hm);

#if 0
    {
        const auto filename
            (str(boost::format("hm-%s.png") % tileId));

        cv::Mat_<cv::Vec3b> tmp(hm.size(), cv::Vec3b());

        auto itmp(tmp.begin());
        for (const auto &px : hm) {
            auto &out(*itmp++);
            out[2] = std::isfinite(px[0]) ? 255 : 0;
            out[1] = std::isfinite(px[1]) ? 255 : 0;
            out[0] = 0;
        }

        cv::imwrite(filename, tmp, {});
    }
#endif
}

} } } // namespace vtslibs::vts::merge
