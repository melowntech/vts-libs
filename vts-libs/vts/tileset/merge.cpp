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

#include <boost/optional.hpp>
#include <boost/utility/in_place_factory.hpp>

#include <opencv2/imgproc/imgproc.hpp>

#include "math/transform.hpp"

#include "imgproc/scanconversion.hpp"

#include "../csconvertor.hpp"

#include "../io.hpp"
#include "../meshop.hpp"

#include "./merge.hpp"
#include "./merge/support.hpp"
#include "./merge/coverage.hpp"

namespace fs = boost::filesystem;

namespace vtslibs { namespace vts {

namespace merge {

class SdMeshConvertor : public MeshVertexConvertor {
public:
    SdMeshConvertor(const Input &input, const NodeInfo &nodeInfo
                    , const TileId &tileId)
        : geoTrafo_(input.coverage2Sd(nodeInfo))
        , geoConv_(nodeInfo.srs()
                   , nodeInfo.referenceFrame().model.physicalSrs)
        , etcNCTrafo_(etcNCTrafo(tileId))
        , coverage2Texture_(input.coverage2Texture())
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

    struct Lazy;

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
};

struct SdMeshConvertor::Lazy {
public:
    Lazy(const Input &input, const NodeInfo &nodeInfo, const TileId &tileId)
        : factory_(input, nodeInfo, tileId)
    {}

    operator const SdMeshConvertor&() const {
        if (!convertor_) { convertor_ = factory_; }
        return *convertor_;
    }

    const SdMeshConvertor& operator()() const { return *this; }

private:
    decltype(boost::in_place
             (std::declval<Input>(), std::declval<NodeInfo>()
              , std::declval<TileId>())) factory_;
    mutable boost::optional<SdMeshConvertor> convertor_;
};

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

class MeshFilter {
public:
    MeshFilter(const SubMesh &original, int submeshIndex
               , const math::Points3 &originalCoverage
               , const Input &input, const Coverage &coverage
               , const MergeOptions &options
               , const SdMeshConvertor::Lazy &sdmc
               , const TileId &diff)
        : original_(original), submeshIndex_(submeshIndex)
        , originalCoverage_(originalCoverage)
        , input_(input), sdmc_(sdmc), diff_(diff)
        , mesh_(result_.mesh), coverageVertices_(result_.projected)
        , vertexMap_(original.vertices.size(), -1)
        , tcMap_(original.tc.size(), -1)
    {
        original_.cloneMetadataInto(mesh_);
        filter(coverage, options.clip);
    }

    void addTo(Output &out);

    EnhancedSubMesh result() const { return result_; }

    operator bool() const { return mesh_.vertices.size(); }

private:
    void filter(const Coverage &coverage, bool clipping) {
        // clipping is off: just pass all faces
        if (!clipping) {
            for (int f(0), ef(original_.faces.size()); f != ef; ++f) {
                addFace(f);
            }
            return;
        }

        const auto id(input_.id());

        // clipping is on: add face only when covered

        // topmost surface: pass as is
        if (coverage.topmost(id)) {
            if (diff_.lod) {
                // deriving from fallback tile -> clip
                const auto clipped(clip(original_, originalCoverage_
                                        , coverageExtents(1.), sdmc_));
                if (clipped) {
                    mesh_ = clipped.mesh;
                    coverageVertices_ = clipped.projected;
                    original_.cloneMetadataInto(mesh_);
                }
                return;
            }

            // keep as-is
            mesh_ = original_;
            return;
        }

        // we have to filter the input mesh
        const auto &cookieCutter(coverage.cookieCutters[id]);

        for (int f(0), ef(original_.faces.size()); f != ef; ++f) {
            const auto covered(coverage.covered
                               (original_.faces[f], originalCoverage_
                                , input_.id()));
            if (!covered) { continue; }

            if (covered) {
                // all fully covered faces are added as-is
                addFace(f);
            } else {
                // partially covered faces are clipped by mesh cookie cutter
                // TODO: clip
                (void) cookieCutter;
            }
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
    const SdMeshConvertor::Lazy &sdmc_;
    const TileId diff_;

    EnhancedSubMesh result_;
    SubMesh &mesh_;
    math::Points3 &coverageVertices_;
    std::vector<int> vertexMap_;
    std::vector<int> tcMap_;
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

    // create output submesh mask
    auto &cm(outMesh.coverageMask);

    auto size(cm.size());

    // rasterize all submeshes
    const auto &faces(mesh.faces);

    // use (position + 1) as a mask color
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

void MeshFilter::addTo(Output &out)
{
    addInputToOutput(out, input_, mesh_, coverageVertices_
                     , submeshIndex_, (1 << (2 * diff_.lod)));
}

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

    LOG(info4) << "clipping single source derived tile: "
               << tileId;

    CsConvertor phys2sd(nodeInfo.referenceFrame().model.physicalSrs
                        , nodeInfo.srs());

    const auto coverageVertices
        (inputCoverageVertices(input, nodeInfo, phys2sd));
    SdMeshConvertor sdmc(input, nodeInfo, localId);

    std::size_t smIndex(0);
    for (const auto &sm : input.mesh()) {
        auto clipped(clip(sm, coverageVertices[smIndex]
                          , coverageExtents(1.), sdmc));

        if (clipped) {
            addInputToOutput(result, input, clipped.mesh
                             , clipped.projected, smIndex
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
        coverage.dump(dumpDir);
    }

    if (const auto *dumpDir = ::getenv("MERGE_COOKIE_CUTTERS_DUMP_DIR")) {
        coverage.dumpCookieCutters(dumpDir);
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

    // process all input tiles from result source (i.e. only those contributing
    // to the tile)

    SurrogateCalculator sc;

    // process all inputs
    for (const auto &input : result.source.mesh) {
        const auto &mesh(input.mesh());

        // get current mesh vertices converted to coverage coordinate system
        const auto coverageVertices
            (inputCoverageVertices(input, nodeInfo, phys2sd));

        auto icoverageVertices(coverageVertices.begin());

        // localize tileId -> used to map fallback content into this tile
        const auto &tileLod(input.tileId().lod);
        const auto localId(local(tileLod, tileId));

        // mesh convertor, lazy (instance is create when needed)
        SdMeshConvertor::Lazy sdmc(input, nodeInfo, localId);

        // traverse all submeshes
        for (int m(0), em(mesh.submeshes.size()); m != em; ++m) {
            const auto &sm(mesh[m]);
            // accumulate new mesh
            MeshFilter mf(sm, m, *icoverageVertices++, input
                          , coverage, options, sdmc, localId);
            if (!mf) {
                // empty result mesh -> nothing to do
                continue;
            }

            // add as is
            mf.addTo(result);
            sc.update(result, input, sm);
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
