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
#include <set>
#include <numeric>

#include <boost/filesystem.hpp>
#include <boost/format.hpp>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "math/transform.hpp"

#include "imgproc/rastermask/cvmat.hpp"
#include "imgproc/scanconversion.hpp"
#include "imgproc/const-raster.hpp"
#include "imgproc/contours.hpp"

#include "../csconvertor.hpp"

#include "./merge.hpp"
#include "../io.hpp"
#include "../meshop.hpp"
#include "../opencv/colors.hpp"

namespace fs = boost::filesystem;

namespace vtslibs { namespace vts {

namespace {

/** Geo coordinates to coverage mask mapping.
 * NB: result is in pixel system: pixel centers have integral indices
 */
inline math::Matrix4 geo2mask(const math::Extents2 &extents
                              , const math::Size2 &gridSize)
{
    math::Matrix4 trafo(boost::numeric::ublas::identity_matrix<double>(4));

    auto es(size(extents));

    // scales
    math::Size2f scale(gridSize.width / es.width
                       , gridSize.height / es.height);

    // scale to grid
    trafo(0, 0) = scale.width;
    trafo(1, 1) = -scale.height;

    // move to origin and also move pixel centers to integral indices
    trafo(0, 3) = -extents.ll(0) * scale.width - 0.5;
    trafo(1, 3) = extents.ur(1) * scale.height - 0.5;

    return trafo;
}

/** Coverage mask mapping to geo coordinates.
 * NB: result is in pixel system: pixel centers have integral indices
 */
inline math::Matrix4 mask2geo(const math::Extents2 &extents
                              , const math::Size2 &gridSize)
{
    math::Matrix4 trafo(boost::numeric::ublas::identity_matrix<double>(4));

    auto es(size(extents));

    // scales
    math::Size2f scale(es.width / gridSize.width
                       , es.height / gridSize.height);

    // scale to grid
    trafo(0, 0) = scale.width;
    trafo(1, 1) = -scale.height;

    // move to origin
    trafo(0, 3) = extents.ll(0) + 0.5 * scale.width;
    trafo(1, 3) = extents.ur(1) - 0.5 * scale.height;

    return trafo;
}

/** Maps external texture coordinates from parent tile into subtile.
 *  Relationship defined by tile id, parent is a root of the tree (i.e. tile id
 *  0-0-0).
 */
inline math::Matrix4 etcNCTrafo(const TileId &id)
{
    math::Matrix4 trafo(boost::numeric::ublas::identity_matrix<double>(4));

    // LOD=0 -> identity
    if (!id.lod) { return trafo; }

    double tileCount(1 << id.lod);

    // number of tiles -> scale
    trafo(0, 0) = tileCount;
    trafo(1, 1) = tileCount;

    // NB: id.x is unsigned -> must cast to double first
    trafo(0, 3) = - double(id.x);
    trafo(1, 3) = (id.y + 1) - tileCount;

    return trafo;
}

/** Maps coverage coordinate into normalized external texture coordinates.
 */
inline math::Matrix4 coverage2EtcTrafo(const math::Size2 &gridSize)
{
    math::Matrix4 trafo(boost::numeric::ublas::identity_matrix<double>(4));

    // scale to normalized range (0, 1)
    trafo(0, 0) = 1.0 / gridSize.width;
    trafo(1, 1) = -1.0 / gridSize.height;

    // shift to proper orientation and cancel halfpixel offset
    trafo(0, 3) = 0.5 / gridSize.width;
    trafo(1, 3) = 1 - 0.5 / gridSize.height;

    return trafo;
}

math::Extents2 coverageExtents(double margin = .0)
{
    const auto grid(Mesh::coverageSize());
    return math::Extents2(-.5 - margin
                          , -.5 - margin
                          , grid.width - .5 + margin
                          , grid.height - .5 + margin);
}

} // namespace

MeshOpInput::MeshOpInput(Id id, const TileSet::Detail &owner
                         , const TileId &tileId
                         , const NodeInfo *nodeInfo, bool lazy)
    : id_(id), tileId_(tileId), owner_(&owner)
    , flags_(owner_->tileIndex.get(tileId))
    , nodeInfo_(nodeInfo)
    , nodeLoaded_(false), node_()
    , mergeableRange_(owner_->properties.lodRange)
{
    prepare(lazy);

    if (auto bottom = owner_->properties.mergeBottomLod) {
        mergeableRange_.max = bottom;
    }
}

MeshOpInput::MeshOpInput(Id id, const TileSet &owner, const TileId &tileId
             , const NodeInfo *nodeInfo, bool lazy)
    : id_(id), tileId_(tileId), owner_(&owner.detail())
    , flags_(owner_->tileIndex.get(tileId))
    , nodeInfo_(nodeInfo)
    , nodeLoaded_(false), node_()
    , mergeableRange_(owner_->properties.lodRange)
{
    prepare(lazy);

    if (auto bottom = owner_->properties.mergeBottomLod) {
        mergeableRange_.max = bottom;
    }
}

bool MeshOpInput::loadNode() const
{
    if (!nodeLoaded_) {
        node_ = owner_->findMetaNode(tileId_);
        nodeLoaded_ = true;
    }
    return node_;
}

void MeshOpInput::prepare(bool lazy)
{
    if (!lazy) {
        // preload stuff if not lazy
        if (loadNode()) {
            if (hasMesh()) { mesh(); }
            if (hasAtlas()) { atlas(); }
            if (hasNavtile()) { navtile(); }
        }
    }

    if (!nodeInfo_) {
        ownNodeInfo_ = NodeInfo(owner_->referenceFrame, tileId_);
        nodeInfo_ = &*ownNodeInfo_;
    }
}

const std::string& MeshOpInput::name() const
{
    return owner_->properties.id;
}

const MetaNode& MeshOpInput::node() const
{
    loadNode();
    return *node_;
}

bool MeshOpInput::watertight() const
{
    return (flags_ & TileIndex::Flag::watertight);
}

bool MeshOpInput::hasMesh() const
{
    return (flags_ & TileIndex::Flag::mesh);
}

bool MeshOpInput::hasAtlas() const
{
    return (flags_ & TileIndex::Flag::atlas);
}

bool MeshOpInput::hasNavtile() const
{
    return (flags_ & TileIndex::Flag::navtile);
}

const Mesh& MeshOpInput::mesh() const
{
    if (!mesh_) {
        mesh_ = owner_->getMesh(tileId_, flags_);
    }

    return *mesh_;
}

const RawAtlas& MeshOpInput::atlas() const
{
    if (!atlas_) {
        atlas_ = boost::in_place();
        owner_->getAtlas(tileId_, *atlas_, flags_);
    }

    return *atlas_;
}

const opencv::NavTile& MeshOpInput::navtile() const
{
    // navtile must have valid node to work properly
    if (!navtile_ && loadNode()) {
        navtile_ = boost::in_place();
        owner_->getNavTile(tileId_, *navtile_, node_);
    }

    return *navtile_;
}

const math::Matrix4 MeshOpInput::sd2Coverage(const NodeInfo &nodeInfo) const
{
    return geo2mask(nodeInfo.extents(), Mesh::coverageSize());
}

const math::Matrix4 MeshOpInput::coverage2Sd(const NodeInfo &nodeInfo) const
{
    return mask2geo(nodeInfo.extents(), Mesh::coverageSize());
}

const math::Matrix4 MeshOpInput::coverage2Texture() const
{
    return coverage2EtcTrafo(Mesh::coverageSize());
}

namespace merge {

/** Returns mesh vertices (vector per submesh) converted to coverage space.
 */
Vertices3List inputCoverageVertices(const Input &input
                                    , const NodeInfo &nodeInfo
                                    , const CsConvertor &conv)
{
    const auto trafo(input.sd2Coverage(nodeInfo));

    const auto &mesh(input.mesh());
    Vertices3List out(mesh.submeshes.size());
    auto iout(out.begin());
    for (const auto &sm : mesh.submeshes) {
        auto &ov(*iout++);
        for (const auto &v : sm.vertices) {
            ov.push_back(transform(trafo, conv(v)));
        }
    }
    return out;
}

Mesh& Output::forceMesh()
{
    if (!mesh) {
        // mesh with empty mask
        mesh = boost::in_place(false);
    }
    return *mesh;
}

RawAtlas& Output::forceAtlas()
{
    if (!atlas) {
        atlas = boost::in_place();
    }
    return *atlas;
}

opencv::NavTile& Output::forceNavtile()
{
    if (!navtile) {
        navtile = boost::in_place();
    }
    return *navtile;
}

namespace {

/** Build uniform source by merging current and parent sources current data have
 *  precedence only tiles with mesh are used.
*/
template <typename Include>
Input::list mergeSource(const Input::list &currentSource
                        , const Input::list &parentSource
                        , Include include)
{
    Input::list source;
    {
        auto icurrentSource(currentSource.begin())
            , ecurrentSource(currentSource.end());
        auto iparentSource(parentSource.begin())
            , eparentSource(parentSource.end());

        // merge head and common content
        while ((icurrentSource != ecurrentSource)
               && (iparentSource != eparentSource))
        {
            const auto &ts(*icurrentSource);
            const auto &ps(*iparentSource);
            if (ts < ps) {
                if (include(ts)) { source.push_back(ts); }
                ++icurrentSource;
            } else if (ps < ts) {
                if (include(ps)) { source.push_back(ps); }
                ++iparentSource;
            } else {
                bool its(include(ts)), ips(include(ps));
                if (its && ips) {
                    if (ts.inMergeableRange()) {
                        source.push_back(ts);
                    } else {
                        source.push_back(ps);
                    }
                } else if (its) {
                    source.push_back(ts);
                } else if (ips) {
                    source.push_back(ps);
                }
                ++icurrentSource;
                ++iparentSource;
            }
        }

        // copy tail (one or another)
        for (; icurrentSource != ecurrentSource; ++icurrentSource) {
            if (include(*icurrentSource)) {
                source.push_back(*icurrentSource);
            }
        }

        for (; iparentSource != eparentSource; ++iparentSource) {
            if (include(*iparentSource)) {
                source.push_back(*iparentSource);
            }
        }
    }
    return source;
}

Input::list filterSources(const Input::list &reference
                          , const Input::list &sources)
{
    Input::list out;
    auto ireference(reference.begin()), ereference(reference.end());
    auto isources(sources.begin()), esources(sources.end());

    // place inputs from sources to output only when present in reference
    while ((ireference != ereference) && (isources != esources)) {
        const auto &r(*ireference);
        const auto &s(*isources);
        if (r < s) {
            ++ireference;
        } else if (s < r) {
            ++isources;
        } else {
            out.push_back(s);
            ++ireference;
            ++isources;
        }
    }
    return out;
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

typedef std::vector<imgproc::Contours> CookieCutters;

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

void dumpCookieCutters(const fs::path &dumpDir, const TileId &tileId
                       , const CookieCutters &cookieCutters
                       , const math::Size2 &size = Mesh::coverageSize())
{
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
      << "     width=\"" << size.width
      << "\" height=\"" << size.height << "\">\n";

    f << "<rect width=\"" << size.width
      << "\" height=\"" << size.height
      << "\" style=\"fill:black;stroke-width:0\" />\n";

    auto colorIndex(0);
    for (const auto contours : cookieCutters) {
        const auto &color(opencv::palette256vec[++colorIndex]);
        for (const auto &contour : contours) {
            if (contour.empty()) { continue; }

            f << "<polygon points=\"";
            for (const auto &point : contour) {
                f << point(0) << ',' << point(1) << ' ';
            }
            f << "\" style=\"fill:none;stroke:" << SvgColor{color}
            << ";stroke-width:.25\" />\n";
        }
    }

    f << "</svg>\n";

    f.close();
}

struct Coverage {
    typedef std::int16_t pixel_type;

    const TileId tileId;
    const Input::list &sources;
    cv::Mat_<pixel_type> coverage;
    bool hasHoles;
    std::vector<bool> indices;
    boost::optional<Input::Id> single;
    CookieCutters cookieCutters;

    Coverage(const TileId &tileId, const NodeInfo &nodeInfo
             , const Input::list &sources)
        : tileId(tileId), sources(sources), hasHoles(false)
        , indices(sources.back().id() + 1, false)
    {
        generateCoverage(nodeInfo);
        analyze();
        findCookieCutters();
    }

    void getSources(Output &output, const Input::list &navtileSource) const {
        for (const auto &input : sources) {
            if (indices[input.id()]) {
                output.source.mesh.push_back(input);
            }
        }
        output.source.navtile
            = filterSources(output.source.mesh, navtileSource);
    }

    std::tuple<bool, bool>
    covered(const Face &face, const math::Points3d &vertices
            , Input::Id id) const
    {
        std::tuple<bool, bool> res(false, false);
        auto &covered(std::get<0>(res));
        auto &inside(std::get<1>(res));

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
                inside = true;

                // TODO: early exit
                if (coverage(y, x) == id) {
                    covered = true;
                }
            });
            if (covered) { return res; }
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

            inside = true;
            if (coverage(y, x) == id) {
                covered = true;
                return res;
            }
        }

        return res;
    }

    void dump(const fs::path &dump, const TileId &tileId) const {
        const auto &colors(opencv::palette256vec);

        cv::Mat_<cv::Vec3b> img(coverage.rows, coverage.cols);

        std::transform(coverage.begin(), coverage.end(), img.begin()
                       , [&](pixel_type v) { return colors[v + 1]; });

        const auto filename
            (dump / str(boost::format("coverage-%s.png") % tileId));
        create_directories(filename.parent_path());
        imwrite(filename.string(), img);
    }

private:
    void generateCoverage(const NodeInfo &nodeInfo) {
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

    void analyze() {
        // BEGIN OPTIMIZATION {
        if (single) {
            // already analyzed during coverage generation
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

        int count(0);
        for (Input::Id id(0), eid(indices.size()); id != eid; ++id) {
            if (!indices[id]) { continue; }
            if (!single) {
                single = id;
            }
            ++count;
        }

        if (count > 1) {
            // more than one dataset -> reset single
            single = boost::none;
        }
    }

    void findCookieCutters() {
        // single case -> no mesh clipping needed, do not find any cookie cutter
        if (single) { return; }

        // prepare workplace
        cv::Mat binary
            (coverage.rows + 2, coverage.cols + 2, CV_8UC1, cv::Scalar(0));
        cv::Mat view(binary, cv::Range(1, coverage.rows + 1)
                     , cv::Range(1, coverage.cols + 1));

        for (const auto &source : sources) {
            if (!indices[source.id()]) {
                cookieCutters.emplace_back();
                continue;
            }

#if 0
            // make room for cookie cutters
            cookieCutters.emplace_back();
            auto &cutters(cookieCutters.back());

            // coverage -> binary image
            binary = cv::Scalar(0);
            const auto id(source.id());
            cv::inRange(coverage, cv::Scalar(id), cv::Scalar(id), view);

            {
                std::vector<std::vector<cv::Point>> contours;
                cv::findContours(binary, contours, CV_RETR_CCOMP
                                 , CV_CHAIN_APPROX_SIMPLE, cv::Point(-1, -1));
                for (const auto &contour : contours) {
                    cutters.emplace_back();
                    auto &cutter(cutters.back());
                    for (const auto &p : contour) {
                        cutter.emplace_back(p.x, p.y);
                    }
                }
            }
#else
            const auto raster
                (imgproc::cvConstRaster<Coverage::pixel_type>(coverage));
            typedef decltype(raster) RasterType;
            const auto id(source.id());
            cookieCutters.push_back
                (imgproc::findContours
                 (raster, [id](const RasterType::value_type &value) {
                     return (value(0) == id);
                 }));
#endif

            LOG(info3) << "Found " << cookieCutters.back().size()
                       << " cookie cutters for source: "
                       << source.name() << ".";
        }

        if (const auto *dumpDir = ::getenv("MERGE_COOKIE_CUTTERS_DUMP_DIR")) {
            dumpCookieCutters(dumpDir, tileId, cookieCutters);
        }
    }
};

class MeshFilter {
public:
    MeshFilter(const SubMesh &original, int submeshIndex
               , const math::Points3 &originalCoverage
               , const Input &input, const Coverage &coverage
               , const MergeOptions &options)
        : original_(original), submeshIndex_(submeshIndex)
        , originalCoverage_(originalCoverage)
        , input_(input)
        , mesh_(result_.mesh), coverageVertices_(result_.projected)
        , vertexMap_(original.vertices.size(), -1)
        , tcMap_(original.tc.size(), -1)
        , incidentTriangles_(0)
    {
        original_.cloneMetadataInto(mesh_);
        filter(coverage, options.clip);
    }

    void addTo(Output &out, double uvAreaScale = 1.0);

    EnhancedSubMesh result() const { return result_; }

    operator bool() const { return mesh_.vertices.size(); }

    std::size_t maxRefinedFaceCount() const {
        return ((original_.faces.size() * mesh_.faces.size())
                / incidentTriangles_);
    }

private:
    void filter(const Coverage &coverage, bool clip) {
        if (!clip) {
            // no clipping -> just pass all faces
            for (int f(0), ef(original_.faces.size()); f != ef; ++f) {
                addFace(f);
                ++incidentTriangles_;
            }
            return;
        }

        // each face covered at least by one pixel is added to new mesh
        for (int f(0), ef(original_.faces.size()); f != ef; ++f) {
            // clipping -> only when covered
            auto covered(coverage.covered
                         (original_.faces[f], originalCoverage_
                          , input_.id()));
            if (std::get<0>(covered)) {
                addFace(f);
            }
            incidentTriangles_ += std::get<1>(covered);
        }
    }

    void addFace(int faceIndex) {
        const auto &of(original_.faces[faceIndex]);

        mesh_.faces.emplace_back
            (addVertex(of(0)), addVertex(of(1)), addVertex(of(2)));

        if (!original_.facesTc.empty()) {
            const auto &oftc(original_.facesTc[faceIndex]);
            mesh_.facesTc.emplace_back
                (addTc(oftc(0)), addTc(oftc(1)), addTc(oftc(2)));
        }
    }

    int addVertex(int i) {
        auto &m(vertexMap_[i]);
        if (m < 0) {
            // new vertex
            m = mesh_.vertices.size();
            mesh_.vertices.push_back(original_.vertices[i]);
            if (!original_.etc.empty()) {
                mesh_.etc.push_back(original_.etc[i]);
            }

            // coverage vertices (if needed)
            coverageVertices_.push_back(originalCoverage_[i]);
        }
        return m;
    }

    int addTc(int i) {
        auto &m(tcMap_[i]);
        if (m < 0) {
            // new vertex
            m = mesh_.tc.size();
            mesh_.tc.push_back(original_.tc[i]);
        }
        return m;
    }

    const SubMesh &original_;
    const int submeshIndex_;
    const math::Points3 originalCoverage_;
    const Input &input_;

    EnhancedSubMesh result_;
    SubMesh &mesh_;
    math::Points3 &coverageVertices_;
    std::vector<int> vertexMap_;
    std::vector<int> tcMap_;
    int incidentTriangles_;
};

void addInputToOutput(Output &out, const Input &input
                      , SubMesh mesh, const math::Points3 &projected
                      , int submeshIndex, double uvAreaScale)
{
    // add atlas if present
    if (input.hasAtlas() && input.atlas().valid(submeshIndex)) {
        out.forceAtlas().add(input.atlas().get(submeshIndex));
    }

    // add submesh
    auto &outMesh(out.forceMesh());
    auto &added(outMesh.add(mesh));

    // update geomExtents
    update(out.geomExtents, geomExtents(projected));

    // set UV area scale
    added.uvAreaScale = uvAreaScale;
    // set surface reference to input tileset index + 1
    added.surfaceReference = input.id() + 1;

    auto &cm(outMesh.coverageMask);

    auto size(cm.size());

    // rasterize all submeshes
    const auto &faces(mesh.faces);

    auto color(outMesh.size());

    // TODO: optimize by using input mask and coverage
    for (const auto &face : faces) {
        std::vector<imgproc::Scanline> scanlines;

        const math::Point3 *tri[3] = {
            &projected[face[0]]
            , &projected[face[1]]
            , &projected[face[2]]
        };

        imgproc::scanConvertTriangle
            (*tri[0], *tri[1], *tri[2], 0, size.height, scanlines);

        for (const auto &sl : scanlines) {
            imgproc::processScanline(sl, 0, size.width
                                         , [&](int x, int y, float)
            {
                cm.set(x, y, color);
            });
        }

        // do one more check in case the triangle is thinner than one pixel
        for (int i = 0; i < 3; ++i) {
            int x(std::round((*tri[i])(0)));
            int y(std::round((*tri[i])(1)));

            if ((x < 0) || (x >= size.width)) {
                continue;
            }
            if ((y < 0) || (y >= size.height)) {
                continue;
            }

            cm.set(x, y, color);
        }
    }
}

void MeshFilter::addTo(Output &out, double uvAreaScale)
{
    addInputToOutput(out, input_, mesh_, coverageVertices_
                     , submeshIndex_, uvAreaScale);
}

class SdMeshConvertor : public MeshVertexConvertor {
public:
    SdMeshConvertor(const Input &input, const NodeInfo &nodeInfo
                    , const TileId &tileId
                    , Lod lodDiff = 0
                    , std::size_t faceLimit = 0)
        : geoTrafo_(input.coverage2Sd(nodeInfo))
        , geoConv_(nodeInfo.srs()
                   , nodeInfo.referenceFrame().model.physicalSrs)
        , etcNCTrafo_(etcNCTrafo(tileId))
        , coverage2Texture_(input.coverage2Texture())
        , lodDiff_(lodDiff), faceLimit_(faceLimit)
    {}

    virtual math::Point3d vertex(const math::Point3d &v) const {
        // point is in node SD SRS
        return geoConv_(transform(geoTrafo_, v));
    }

    virtual math::Point2d etc(const math::Point3d &v) const {
        // point is in projected space (i.e. in coverage raster)
        auto tmp(transform(coverage2Texture_, v));
        return math::Point2d(tmp(0), tmp(1));
    }

    virtual math::Point2d etc(const math::Point2d &v) const {
        // point is in the input's texture coordinates system
        return transform(etcNCTrafo_, v);
    }

    virtual std::size_t refineToFaceCount(std::size_t current) const
    {
        // no refinement -> use current count
        if (!lodDiff_) { return current; }

        // scale current number of faces by 4^lodDiff (i.e. 2^(2 * lodDiff))
        // Limit lodDiff to 8;
        int exponent(2 * std::min(lodDiff_, Lod(8)));
        // scale current number of faces
        std::size_t scaled(current << exponent);
        // limit scaled number of faces by original number of faces in the mesh
        return std::min(scaled, faceLimit_);
    }

    void faceLimit(std::size_t value) { faceLimit_ = value; }

private:
    /** Linear transformation from local coverage coordinates to node's SD SRS.
     */
    math::Matrix4 geoTrafo_;

    /** Convertor between node's SD SRS and reference frame's physical SRS.
     */
    CsConvertor geoConv_;

    /** Converts external texture coordinates between fallback tile and current
     *  tile.
     */
    math::Matrix4 etcNCTrafo_;

    /** Converts between coverage coordinates and normalized external texture
     *  cooridnates.
     */
    math::Matrix4 coverage2Texture_;

    /** Difference between current LOD and tile's original lod.
     */
    Lod lodDiff_;

    /** Limit number of faces after refinement to given number.
     */
    std::size_t faceLimit_;
};

void renderNavtile(cv::Mat &nt, cv::Mat &ntCoverage
                   , const opencv::NavTile &navtile)
{
    auto coverage(renderCoverage(navtile));
    navtile.data().copyTo(nt, coverage);
    coverage.copyTo(ntCoverage, coverage);
}

/** This function could eat a lot of memory since it scales whole tile instead
 *  of just sub-range.
 *
 *  TODO: state limit on lod difference and scale in multiple steps
 */
void renderNavtile(cv::Mat &nt, cv::Mat &ntCoverage
                   , const TileId &localId
                   , const opencv::NavTile &navtile)
{
    constexpr Lod processLodDifference(3);

    // render coverage as usual
    auto coverage(renderCoverage(navtile));

    typedef std::tuple<cv::Mat, cv::Mat> NavData;

    /** Scales navtile data/coverage mask in grid registration
     */
    auto scaleNavtile([&](const TileId &localId, const NavData &in)
                      -> NavData
    {
        // scale size of data in grid registration
        const auto nts(NavTile::size());
        cv::Size scaledSize((nts.width - 1) * (1 << localId.lod) + 1
                            , (nts.height - 1) * (1 << localId.lod) + 1);

        // create column and row ranges to grab subtile
        cv::Range cr((nts.width - 1) * localId.x, 0);
        cr.end = cr.start + nts.width;

        cv::Range rr((nts.height - 1) * localId.y, 0);
        rr.end = rr.start + nts.height;

        // this helper function scales tile data in grid registrion and returns
        // sub-image at given indices
        auto resizeAndCrop([&](const cv::Mat &in) -> cv::Mat
        {
            // placeholder
            cv::Mat tmp;
            // resize whole image
            cv::resize(in, tmp, scaledSize, 0.0, 0.0, cv::INTER_LINEAR);
            // clone subrange
            return cv::Mat(tmp, rr, cr).clone();
        });

        // create source data for tile
        return NavData(resizeAndCrop(std::get<0>(in))
                       , resizeAndCrop(std::get<1>(in)));
    });

    // Splits tile id into two tile id's:
    //
    //     * parent of input tileId that is at most diff LODs from root (return
    //     * value)
    //
    //     * new tileId under whitch this parent as a root (modified argument)
    //
    auto splitId([&](Lod diff, TileId &id) -> TileId
    {
        // too shallow tree -> identity
        if (id.lod <= diff) {
            auto nid(id);
            id = {};
            return nid;
        }

        // deeper tree

        // create new root
        const auto rest(id.lod - diff);
        TileId nid(diff, id.x >> rest, id.y >> rest);

        // re-root id under new root
        id = local(diff, id);

        // done
        return nid;
    });

    // process data in parts
    NavData nd(navtile.data(), coverage);
    for (auto lid(localId); lid.lod; ) {
        const auto id(splitId(processLodDifference, lid));
        nd = scaleNavtile(id, nd);
    }

    // apply data and mask
    const auto &subData(std::get<0>(nd));
    const auto &subCoverage(std::get<1>(nd));
    subData.copyTo(nt, subCoverage);
    subCoverage.copyTo(ntCoverage, subCoverage);
}

void mergeNavtile(Output &output)
{
    auto nt(opencv::NavTile::createData());
    cv::Mat mask(nt.rows, nt.cols, CV_8U, cv::Scalar(0));
    for (const auto &input : output.source.navtile) {
        if (output.derived(input)) {
            renderNavtile(nt, mask, local(input.tileId().lod, output.tileId)
                          , input.navtile());
        } else {
            renderNavtile(nt, mask, input.navtile());
        }
    }

    output.forceNavtile().data(nt, mask);
}

Output singleSourced(const TileId &tileId, const NodeInfo &nodeInfo
                     , const Input &input, const Input::list &navtileSource
                     , bool generateNavtile)
{
    Output result(tileId, input, navtileSource);
    if (input.tileId().lod == tileId.lod) {
        // as is -> copy
        result.mesh = input.mesh();
        result.geomExtents = input.node().geomExtents;

        // update surface references of all submeshes
        for (auto &sm : *result.mesh) {
            sm.surfaceReference = input.id() + 1;
        }

        if (input.hasAtlas()) { result.atlas = input.atlas(); }
        if (input.hasNavtile()) { result.navtile = input.navtile(); }
        if (generateNavtile) { mergeNavtile(result); }
        return result;
    }

    // derived tile -> cut out
    const auto localId(local(input.tileId().lod, tileId));

    // check if subtile is covered in input (do not generate empty tile and
    // do not let empty subtile fall through)
    if (! input.mesh().coverageMask.get(localId.lod, localId.x, localId.y) ) {
        return Output(tileId);
    }

    // clip source mesh/navtile

    CsConvertor phys2sd(nodeInfo.referenceFrame().model.physicalSrs
                        , nodeInfo.srs());

    const auto coverageVertices
        (inputCoverageVertices(input, nodeInfo, phys2sd));
    SdMeshConvertor sdmc(input, nodeInfo, localId);

    std::size_t smIndex(0);
    std::size_t outSmIndex(0);
    for (const auto &sm : input.mesh()) {
        auto refined
            (clipAndRefine({ sm, coverageVertices[smIndex] }
                           , coverageExtents(1.), sdmc));

        if (refined) {
            addInputToOutput(result, input, refined.mesh
                             , refined.projected, outSmIndex++
                             , (1 << (2 * localId.lod)));
        }

        ++smIndex;
    }

    if (generateNavtile) { mergeNavtile(result); }

    // set surroggate
    // TODO: filter from vicinity of source tile
    result.geomExtents.surrogate = input.node().geomExtents.surrogate;

    // done
    return result;
}

class SurrogateCalculator {
public:
    SurrogateCalculator()
        : sum_(), weightSum_()
    {}

    /** Updates surrogate calculation with last computed mesh
     *
     * \param result generated output, last submesh is used
     * \param input input used to generate mesh, metanode is used
     * \param source source submesh from input
     */
    void update(const Output &result, const MeshOpInput &input
                , const SubMesh &source)
    {
        // get laat mesh
        const auto *outMesh(result.getMesh());
        if (!outMesh || outMesh->empty()) { return; }

        const auto weight(area3d(outMesh->submeshes.back())
                          / area3d(source));

        sum_ += weight * input.node().geomExtents.surrogate;
        weightSum_ += weight;
    }

    double surrogate() const {
        return sum_ / weightSum_;
    }

private:
    double sum_;
    double weightSum_;
};

} // namespace

Output mergeTile(const TileId &tileId
                 , const NodeInfo &nodeInfo
                 , const Input::list &currentSource
                 , const TileSource &parentSource
                 , const MergeConstraints &constraints
                 , const MergeOptions &options)
{
    // merge sources for meshes and navtiles
    auto source
        (mergeSource
         (currentSource, parentSource.mesh
          , [](const Input &input) { return input.hasMesh(); }));
    auto navtileSource
         (mergeSource
          (currentSource, parentSource.navtile
           , [](const Input &input) { return input.hasNavtile(); }));

    if (!constraints.generable()) {
        // just sources
        LOG(info1) << "(merge) Constraits prohibit generation.";
        return Output(tileId, source, navtileSource);
    }

    // from here, all input tiles have geometry -> no need to check for mesh
    // presence

    if (source.empty()) {
        LOG(info1) << "(merge) No sources at all.";
        return Output(tileId);
    }

    LOG(info1)
        << "(merge) Sources to merge: "
        << utility::LManip([&](std::ostream &os) -> void
           {
               bool first = true;
               for (const auto &src : source) {
                   if (!first) { os << ", "; }
                   os << src.name();
                   if (src.tileId() != tileId) {
                       os << "(" << src.tileId() << ")";
                   }
                   first = false;
               }
           }) << ".";

    if ((source.size() == 1)) {
        Output result(tileId, source, navtileSource);
        if (!constraints.feasible(result)) { return result; }

        // just one source
        return singleSourced(tileId, nodeInfo, source.front()
                             , filterSources(source, navtileSource)
                             , constraints.generateNavtile());
    }

    // merge result
    Output result(tileId);

    // analyze coverage
    Coverage coverage(tileId, nodeInfo, source);

    if (const auto *dumpDir = ::getenv("MERGE_MASK_DUMP_DIR")) {
        coverage.dump(dumpDir, tileId);
    }

    // get contributing tile sets
    coverage.getSources(result, navtileSource);

    if (!constraints.feasible(result)) {
        // nothing to merge
        return result;
    }

    if (coverage.single) {
        // single source
        if (*coverage.single < 0) {
            // nothing at all
            return result;
        }

        // process single source
        return singleSourced(tileId, nodeInfo, result.source.mesh.front()
                             , result.source.navtile
                             , constraints.generateNavtile());
    }

    // merge meshes
    CsConvertor phys2sd(nodeInfo.referenceFrame().model.physicalSrs
                        , nodeInfo.srs());

    // compute bottom (maximum) lod from all inputs
    auto bottomLod(std::accumulate
                   (result.source.mesh.begin()
                    , result.source.mesh.end()
                    , Lod(0), [](Lod lod, const MeshOpInput &i)
                    {
                        return std::max(lod, i.tileId().lod);
                    }));

    // process all input tiles from result source (i.e. only those contributing
    // to the tile)

    SurrogateCalculator sc;

    for (const auto &input : result.source.mesh) {
        const auto &mesh(input.mesh());

        // get current mesh vertices converted to coverage coordinate system
        const auto coverageVertices
            (inputCoverageVertices(input, nodeInfo, phys2sd));

        auto icoverageVertices(coverageVertices.begin());

        // localize tileId -> used to map fallback content into this tile
        const auto &tileLod(input.tileId().lod);
        const auto localId(local(tileLod, tileId));

        // traverse all submeshes
        for (int m(0), em(mesh.submeshes.size()); m != em; ++m) {
            const auto &sm(mesh[m]);
            // accumulate new mesh
            MeshFilter mf(sm, m, *icoverageVertices++, input
                          , coverage, options);
            if (!mf) {
                // empty result mesh -> nothing to do
                continue;
            }

            if (!localId.lod) {
                // add as is
                mf.addTo(result);
                sc.update(result, input, sm);
                continue;
            }

            // fallback mesh: we have to clip and refine accumulated
            // mesh and process again
            auto refined
                (clipAndRefine
                 (mf.result(), coverageExtents(1.)
                  , SdMeshConvertor(input, nodeInfo, localId
                                    , (bottomLod - tileLod)
                                    , mf.maxRefinedFaceCount())));

            MeshFilter rmf(refined.mesh, m, refined.projected
                           , input, coverage, options);
            if (rmf) {
                // add refined
                rmf.addTo(result, (1 << (2 * localId.lod)));
                sc.update(result, input, sm);
            }
        }
    }

    // generate navtile if asked to
    if (constraints.generateNavtile()) { mergeNavtile(result); }

    // set surrogate
    result.geomExtents.surrogate = sc.surrogate();

    return result;
}

Tile Output::tile(int textureQuality)
{
    Tile tile;

    if (textureQuality && mesh) {
        // we have mesh and should generate textures, try to optimize it

        // wrap mesh and atlas in shared pointers
        Mesh::pointer m;
        RawAtlas::pointer a;
        if (atlas) { a.reset(&*atlas, [](void*) {}); }
        if (mesh) { m.reset(&*mesh, [](void*) {}); }

        if (textureQuality) {
            // optimize
            auto optimized(mergeSubmeshes(tileId, m, a, textureQuality));
            // assign output
            tile.mesh = std::get<0>(optimized);
            tile.atlas = std::get<1>(optimized);
        } else {
            // no optimization
            tile.mesh = m;
            tile.atlas = a;
        }
    } else {
        // nothing, just wrap members into output
        if (mesh) { tile.mesh.reset(&*mesh, [](void*) {}); }
        if (atlas) { tile.atlas.reset(&*atlas, [](void*) {}); }
    }

    if (navtile) { tile.navtile.reset(&*navtile, [](void*) {}); }

    // join all credits from tile mesh source
    for (const auto &src : source.mesh) {
        const auto &sCredits(src.node().credits());
        tile.credits.insert(sCredits.begin(), sCredits.end());
    }

    // update geom extents
    tile.geomExtents = geomExtents;

    return tile;
}

} } } // namespace vtslibs::vts::merge
