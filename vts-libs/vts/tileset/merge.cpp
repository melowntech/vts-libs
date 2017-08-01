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

#include <opencv2/imgproc/imgproc.hpp>

#include "math/transform.hpp"

#include "imgproc/scanconversion.hpp"

#include "../io.hpp"

#include "./merge.hpp"
#include "./merge/support.hpp"
#include "./merge/coverage.hpp"

namespace fs = boost::filesystem;

namespace vtslibs { namespace vts {

namespace merge {

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

struct IntermediateOutput {
    std::reference_wrapper<const Input> input;
    EnhancedSubMesh mesh;
    int submeshIndex;
    double uvAreaScale;

    typedef std::vector<IntermediateOutput> list;

    IntermediateOutput(const Input &input, EnhancedSubMesh mesh
                       , int submeshIndex, double uvAreaScale)
        : input(input), mesh(std::move(mesh)), submeshIndex(submeshIndex)
        , uvAreaScale(uvAreaScale)
    {}

    IntermediateOutput(const Input &input, SubMesh sm, math::Points3 projected
                       , int submeshIndex, double uvAreaScale)
        : input(input), mesh(std::move(sm), std::move(projected))
        , submeshIndex(submeshIndex)
        , uvAreaScale(uvAreaScale)
    {}
};

class MeshFilter {
public:
    MeshFilter(const SubMesh &original, int submeshIndex
               , const math::Points3 &originalCoverage
               , const Input &input, Coverage &coverage
               , const MergeOptions &options
               , const SdMeshConvertor::Lazy &sdmc
               , const TileId &diff);

    void addTo(IntermediateOutput::list &out);

    operator bool() const {
        return !(keep_ ? original_.empty() : mesh_.empty());
    }

private:
    void filter(Coverage &coverage, bool clipping);

    void updateHeightmap(Coverage &coverage, const Faces &faces
                         , const math::Points3 &vertices);

    void complexClip(Coverage &coverage);

    const SubMesh &original_;
    const int submeshIndex_;
    const math::Points3 originalCoverage_;
    const Input &input_;
    const SdMeshConvertor::Lazy &sdmc_;
    const TileId diff_;

    SubMesh mesh_;
    math::Points3 coverageVertices_;
    bool keep_;
};

MeshFilter::MeshFilter(const SubMesh &original, int submeshIndex
                       , const math::Points3 &originalCoverage
                       , const Input &input, Coverage &coverage
                       , const MergeOptions &options
                       , const SdMeshConvertor::Lazy &sdmc
                       , const TileId &diff)
    : original_(original), submeshIndex_(submeshIndex)
    , originalCoverage_(originalCoverage)
    , input_(input), sdmc_(sdmc), diff_(diff)
    , keep_(false)
{
    // clipping is off: just pass all faces
    if (!options.clip) {
        keep_ = true;
        updateHeightmap(coverage, original_.faces, originalCoverage_);
        return;
    }

    // clipping is on: add face only when covered

    // topmost surface: pass as is
    if (coverage.topmost(input_.id())) {
        if (diff_.lod) {
            // deriving from fallback tile -> clip
            const auto clipped(clip(original_, originalCoverage_
                                    , coverageExtents(1.), sdmc_));
            if (clipped) {
                mesh_ = clipped.mesh;
                coverageVertices_ = clipped.projected;
                original_.cloneMetadataInto(mesh_);
                updateHeightmap(coverage, mesh_.faces, coverageVertices_);
            }
            return;
        }

        // keep as-is
        keep_ = true;
        updateHeightmap(coverage, original_.faces, originalCoverage_);
        return;
    }

    complexClip(coverage);
}

void MeshFilter::addTo(IntermediateOutput::list &out) {
    if (keep_) {
        out.emplace_back(input_, original_, originalCoverage_
                         , submeshIndex_, (1 << (2 * diff_.lod)));
    } else {
        out.emplace_back(input_, mesh_, coverageVertices_
                         , submeshIndex_, (1 << (2 * diff_.lod)));
    }
}

void MeshFilter::updateHeightmap(Coverage &coverage, const Faces &faces
                                 , const math::Points3 &vertices)
{
    const auto id(input_.id());
    for (const auto &face : faces) {
        coverage.covered(face, vertices, id);
    }
}

void MeshFilter::complexClip(Coverage &coverage)
{
    struct Clipper {
        Clipper(MeshFilter &mf, const SdMeshConvertor *sdmc)
            : mf(mf), sdmc(sdmc)
            , vertexMap(mf.original_.vertices.size(), -1)
            , tcMap(mf.original_.vertices.size(), -1)
        {}

        void addFace(int faceIndex) {
            const auto &of(mf.original_.faces[faceIndex]);

            mf.mesh_.faces.emplace_back
                (addVertex(of(0)), addVertex(of(1)), addVertex(of(2)));

            if (!mf.original_.facesTc.empty()) {
                const auto &oftc(mf.original_.facesTc[faceIndex]);
                mf.mesh_.facesTc.emplace_back
                    (addTc(oftc(0)), addTc(oftc(1)), addTc(oftc(2)));
            }
        }

        int addVertex(int i) {
            auto &m(vertexMap[i]);
            if (m < 0) {
                // new vertex
                m = mf.mesh_.vertices.size();
                mf.mesh_.vertices.push_back(mf.original_.vertices[i]);
                if (!mf.original_.etc.empty()) {
                    mf.mesh_.etc.push_back
                        (sdmc
                         ? sdmc->etc(mf.original_.etc[i])
                         : mf.original_.etc[i]);
                }

                // coverage vertices (if needed)
                mf.coverageVertices_.push_back(mf.originalCoverage_[i]);
            }
            return m;
        }

        int addTc(int i) {
            auto &m(tcMap[i]);
            if (m < 0) {
                // new vertex
                m = mf.mesh_.tc.size();
                mf.mesh_.tc.push_back(mf.original_.tc[i]);
            }
            return m;
        }

        MeshFilter &mf;
        const SdMeshConvertor *sdmc;
        std::vector<int> vertexMap;
        std::vector<int> tcMap;
    };

    Clipper clipper(*this, diff_.lod ? &sdmc_() : nullptr);

    // building new mesh -> we have to clone metadata from original
    original_.cloneMetadataInto(mesh_);

    const auto id(input_.id());

    // we have to filter the input mesh
    const auto &cookieCutter(coverage.cookieCutters[id]);

    for (int f(0), ef(original_.faces.size()); f != ef; ++f) {
        const auto covered
            (coverage.covered(original_.faces[f], originalCoverage_, id));

        // not covered at all -> skip
        if (!covered) { continue; }

        if (covered) {
            // all fully covered faces are added as-is
            clipper.addFace(f);
        } else {
            // partially covered faces are clipped by mesh cookie cutter
            // TODO: clip
            (void) cookieCutter;
        }
    }
}

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

void toOutput(Output &out, const IntermediateOutput::list &imo)
{
    for (const auto &io : imo) {
        addInputToOutput(out, io.input, io.mesh.mesh, io.mesh.projected
                         , io.submeshIndex, io.uvAreaScale);
    }
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

    CsConvertor phys2sd(nodeInfo.referenceFrame().model.physicalSrs
                        , nodeInfo.srs());

    const auto coverageVertices
        (inputCoverageVertices(input, nodeInfo, phys2sd));
    SdMeshConvertor sdmc(nodeInfo, localId);

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
    IntermediateOutput::list imo;

    // mesh convertor (for this tile)
    SdMeshConvertor thisSdmc(nodeInfo);

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

        // mesh vertex convertor: use convertor for this tile or create lazy
        // instance if we are using fallback data (deriving subtile)
        auto sdmc(localId.lod
                  ? SdMeshConvertor::Lazy(nodeInfo, localId)
                  : SdMeshConvertor::Lazy(thisSdmc));

        // traverse all submeshes
        for (int m(0), em(mesh.submeshes.size()); m != em; ++m) {
            const auto &sm(mesh[m]);

            // filter mesh
            MeshFilter mf(sm, m, *icoverageVertices++, input
                          , coverage, options, sdmc, localId);

            if (mf) {
                // add to intermediate result
                mf.addTo(imo);

                // TODO: move surrogate update after skirting?
                // FIXME: use only cut mesh, not input mesh?
                sc.update(result, input, sm);
            }
        }
    }

    // add skirt
    if (1) {
        for (auto &io : imo) {
            addSkirt(io.mesh, thisSdmc
                     , [&](const math::Point3d &p) -> math::Point3d
            {
                (void) p;
                // TODO: implement me
                return math::Point3d(0, 0, -10.0);
            });
        }
    }

    // pass intermediate result to output
    toOutput(result, imo);

    // generate navtile if asked to
    if (constraints.generateNavtile()) { mergeNavtile(result); }

    // set surrogate
    result.geomExtents.surrogate = sc.surrogate();

    return result;
}

} } } // namespace vtslibs::vts::merge
