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

#include "geometry/nonconvexclip.hpp"

#include "imgproc/scanconversion.hpp"

#include "../io.hpp"

#include "merge.hpp"
#include "merge/support.hpp"
#include "merge/coverage.hpp"

namespace fs = boost::filesystem;

namespace vtslibs { namespace vts {

namespace merge {

namespace {

const auto *MERGE_MASK_DUMP_DIR(::getenv("MERGE_MASK_DUMP_DIR"));
const auto *MERGE_COOKIE_CUTTERS_DUMP_DIR
    (::getenv("MERGE_COOKIE_CUTTERS_DUMP_DIR"));

/** Build uniform source by merging current and parent sources current data have
 *  precedence. Only tiles with mesh are used.
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
    struct Copier;
    friend struct Copier;

    struct Clipper;
    friend struct Clipper;

    void filter(Coverage &coverage, bool clipping);

    void updateHeightmap(Coverage &coverage, const Faces &faces
                         , const math::Points3 &vertices);

    void complexClip(Coverage &coverage);

    void simpleClip(Coverage &coverage);

    const MergeOptions &options_;
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
    : options_(options), original_(original), submeshIndex_(submeshIndex)
    , originalCoverage_(originalCoverage)
    , input_(input), sdmc_(sdmc), diff_(diff)
    , keep_(false)
{
    // topmost surface or pure composition -> pass as is
    if ((options_.glueMode == GlueMode::compose)
        || (coverage.topmost(input_.id())))
    {
        if (diff_.lod) {
            // deriving from fallback tile -> clip
            const auto clipped(clip(original_, originalCoverage_
                                    , coverageExtents(options_.safetyMargin)
                                    , sdmc_));
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

    switch (options_.glueMode) {
    case GlueMode::compose: break;
    case GlueMode::simpleClip: simpleClip(coverage); break;
    case GlueMode::coverageContour: complexClip(coverage); break;
    }
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

typedef std::vector<int> IndexMap;

struct MeshFilter::Copier : boost::noncopyable {
    Copier(MeshFilter &mf, const SdMeshConvertor *sdmc = nullptr)
        : sdmc(sdmc)
        , im(mf.original_), ipv(mf.originalCoverage_)
        , om(mf.mesh_), opv(mf.coverageVertices_)
        , hasTc(!im.facesTc.empty()), hasEtc(!im.etc.empty())
        , vertexMap(im.vertices.size(), -1)
        , tcMap(im.tc.size(), -1)
    {}

    Copier(MeshFilter &mf, const SdMeshConvertor *sdmc
            , const SubMesh &im, const math::Points3 &ipv)
        : sdmc(sdmc)
        , im(im), ipv(ipv)
        , om(mf.mesh_), opv(mf.coverageVertices_)
        , hasTc(!im.facesTc.empty()), hasEtc(!im.etc.empty())
        , vertexMap(im.vertices.size(), -1)
        , tcMap(im.tc.size(), -1)
    {}

    void operator()(Coverage &coverage, Input::Id id) {
        // every hit face is passed to output mesh as-is
        for (int f(0), ef(im.faces.size()); f != ef; ++f) {
            if (coverage.hit(im.faces[f], ipv, id).covered) { addFace(f); }
        }
    }

    void addFace(int faceIndex) {
        const auto &of(im.faces[faceIndex]);

        om.faces.emplace_back
            (addVertex(of(0)), addVertex(of(1)), addVertex(of(2)));

        if (hasTc) {
            const auto &oftc(im.facesTc[faceIndex]);
            om.facesTc.emplace_back
                (addTc(oftc(0)), addTc(oftc(1)), addTc(oftc(2)));
        }
    }

    int addVertex(int i) {
        auto &m(vertexMap[i]);
        if (m < 0) {
            // new vertex
            m = om.vertices.size();
            om.vertices.push_back(im.vertices[i]);
            if (hasEtc) {
                om.etc.push_back
                    (sdmc
                     ? sdmc->etc(im.etc[i])
                     : im.etc[i]);
            }

            // coverage vertices (if needed)
            opv.push_back(ipv[i]);
        }
        return m;
    }

    int addTc(int i) {
        auto &m(tcMap[i]);
        if (m < 0) {
            // new vertex
            m = om.tc.size();
            om.tc.push_back(im.tc[i]);
        }
        return m;
    }

    const SdMeshConvertor *sdmc;

    const SubMesh &im;
    const math::Points3 &ipv;
    SubMesh &om;
    math::Points3 &opv;
    bool hasTc;
    bool hasEtc;

    IndexMap vertexMap;
    IndexMap tcMap;
};

struct MeshFilter::Clipper : boost::noncopyable {
    Clipper(Copier &copier, const SdMeshConvertor &sdmc
            , const std::vector<int> &partial
            , const math::MultiPolygon &clipRings)
        : sdmc(sdmc), im(copier.im), ipv(copier.ipv)
        , om(copier.om), opv(copier.opv)
        , hasTc(copier.hasTc), hasEtc(copier.hasEtc)
    {
        // prefill point2index mappings with already used points
        for (auto f : partial) {
            for (auto i : im.faces[f]) {
                auto o(copier.vertexMap[i]);
                if (o > -1) {
                    vertexIndices
                        .insert(Vertex2Index::value_type(opv[o], o));
                }
            }

            if (!hasTc) { continue; }

            for (auto i : im.facesTc[f]) {
                auto o(copier.tcMap[i]);
                if (o > -1) {
                    textureIndices
                        .insert(Texture2Index::value_type(om.tc[o], o));
                }
            }
        }

        if (hasTc) {
            for (auto f : partial) {
                clip(im.faces[f], im.facesTc[f], clipRings);
            }
        } else {
            for (auto f : partial) {
                clip(im.faces[f], clipRings);
            }
        }
    }

    int addVertex(const math::Point3d &v) {
        auto fvertexIndices(vertexIndices.find(v));
        if (fvertexIndices != vertexIndices.end()) {
            return fvertexIndices->second;
        }

        const auto index(opv.size());

        vertexIndices.insert(Vertex2Index::value_type(v, index));

        opv.push_back(v);
        om.vertices.push_back(sdmc.vertex(v));
        if (hasEtc) { om.etc.push_back(sdmc.etc(v)); }

        return index;
    }

    int addTc(const math::Point2d &t) {
        auto ftextureIndices(textureIndices.find(t));
        if (ftextureIndices != textureIndices.end()) {
            return ftextureIndices->second;
        }

        const auto index(om.tc.size());

        textureIndices.insert(Texture2Index::value_type(t, index));

        om.tc.push_back(t);
        return index;
    }

    void addFace(const math::Triangle3d &t) {
        om.faces.emplace_back
            (addVertex(t[0]), addVertex(t[1]), addVertex(t[2]));
    }

    void addFace(const math::Triangle2d &t) {
        om.facesTc.emplace_back(addTc(t[0]), addTc(t[1]), addTc(t[2]));
    }

    void clip(const Face &face, const math::MultiPolygon &clipRings) {
        const auto clipped
            (geometry::clipTriangleNonconvex
             ({{ ipv[face(0)], ipv[face(1)], ipv[face(2)] }}
                 , clipRings));

        if (clipped.empty()) { return; }

        for (const auto &t : clipped) { addFace(t); }
    }

    void clip(const Face &face, const Face &faceTc
              , const math::MultiPolygon &clipRings)
    {
        math::Triangles3d clipped;
        math::Triangles2d clippedTc;

        std::tie(clipped, clippedTc) =
            (geometry::clipTexturedTriangleNonconvex
             ({{ ipv[face(0)], ipv[face(1)], ipv[face(2)] }}
              , {{ im.tc[faceTc(0)], im.tc[faceTc(1)], im.tc[faceTc(2)] }}
              , clipRings));

        if (clipped.empty()) { return; }

        auto iclippedTc(clippedTc.begin());
        for (const auto &t : clipped) {
            const auto &ttc(*iclippedTc++);
            addFace(t);
            addFace(ttc);
        }
    }

    const SdMeshConvertor &sdmc;

    const SubMesh &im;
    const math::Points3 &ipv;
    SubMesh &om;
    math::Points3 &opv;

    bool hasTc;
    bool hasEtc;

    typedef std::map<math::Point3d, int> Vertex2Index;
    typedef std::map<math::Point2d, int> Texture2Index;

    Vertex2Index vertexIndices;
    Texture2Index textureIndices;
};

void MeshFilter::complexClip(Coverage &coverage)
{
    Copier copier(*this, diff_.lod ? &sdmc_() : nullptr);

    // building new mesh -> we have to clone metadata from original
    original_.cloneMetadataInto(mesh_);

    const auto id(input_.id());

    std::vector<int> partial;

    const auto &cookieCutter(coverage.cookieCutters[id]);

    for (int f(0), ef(original_.faces.size()); f != ef; ++f) {
        const auto covered
            (coverage.covered(original_.faces[f], originalCoverage_, id));

        // not covered at all -> skip
        if (!covered) { continue; }

        if (covered) {
            // every fully covered face is added as-is
            copier.addFace(f);
        } else if (cookieCutter) {
            partial.push_back(f);
        }
    }

    if (partial.empty()) { return; }
    Clipper(copier, sdmc_, partial, cookieCutter.rings);
}

void MeshFilter::simpleClip(Coverage &coverage)
{
    // building new mesh -> we have to clone metadata from original
    original_.cloneMetadataInto(mesh_);

    const auto id(input_.id());

    // same LOD -> just pass all incident faces
    if (!diff_.lod) { return Copier(*this)(coverage, id); }

    // LOD difference => we need to refine and clip source mesh

    // accumulate inside faces
    int coveredFaces(0);
    int insideFaces(0);
    for (int f(0), ef(original_.faces.size()); f != ef; ++f) {
        const auto hit(coverage.hit(original_.faces[f]
                                    , originalCoverage_, id));
        coveredFaces += hit.covered;
        insideFaces += hit.inside;
    }

    // sanity check
    if (!insideFaces) { return; }

    struct MVC : public MeshVertexConvertor {
        const SdMeshConvertor &sdmc;
        const Lod lodDiff;
        const std::size_t faceLimit;

        MVC(const SdMeshConvertor &sdmc, Lod lodDiff, std::size_t faceLimit)
            : sdmc(sdmc), lodDiff(lodDiff), faceLimit(faceLimit)
        {}

        virtual math::Point3d vertex(const math::Point3d &v) const {
            return sdmc.vertex(v);
        }

        virtual math::Point2d etc(const math::Point3d &v) const {
            return sdmc.etc(v);
        }

        virtual math::Point2d etc(const math::Point2d &v) const {
            return sdmc.etc(v);
        }

        virtual std::size_t refineToFaceCount(std::size_t current) const {
            // scale current number of faces by 4^lodDiff (i.e. 2^(2 * lodDiff))
            // Limit lodDiff to 8;
            const int exponent(2 * std::min(lodDiff, Lod(8)));
            // scale current number of faces
            const std::size_t scaled(current << exponent);
            // limit scaled number of faces by original number of faces in the
            // mesh
            return std::min(scaled, faceLimit);
        }
    };

    // compute generated face limit
    const auto maxRefinedFaceCount
        ((original_.faces.size() * coveredFaces) / insideFaces);

    // clip and refine mesh
    auto refined(clipAndRefine
                 ({ original_, originalCoverage_ }
                  , coverageExtents(options_.safetyMargin)
                  , MVC(sdmc_(), diff_.lod, maxRefinedFaceCount)));

    if (refined) {
        // anything left? add to output
        Copier(*this, nullptr, refined.mesh, refined.projected)(coverage, id);
    }
}

void addInputToOutput(const MergeOptions &options
                      , Output &out, const Input &input
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

    const auto margin(options.safetyMargin);
    const math::Size2 sizeMargin(size.width + margin, size.height + margin);

    for (const auto &face : faces) {
        std::vector<imgproc::Scanline> scanlines;

        const math::Point3 *tri[3] = {
            &projected[face[0]]
            , &projected[face[1]]
            , &projected[face[2]]
        };

        imgproc::scanConvertTriangle
            (*tri[0], *tri[1], *tri[2], margin, sizeMargin.height, scanlines);

        for (const auto &sl : scanlines) {
            imgproc::processScanline(sl, margin, sizeMargin.width
                                     , [&](int x, int y, float)
            {
                cm.set(x - margin, y - margin, color);
            });
        }

        // do one more check in case the triangle is thinner than one pixel
        for (int i = 0; i < 3; ++i) {
            int x(std::floor((*tri[i])(0)) - margin);
            int y(std::floor((*tri[i])(1)) - margin);

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

void toOutput(const MergeOptions &options, Output &out
              , const IntermediateOutput::list &imo)
{
    for (const auto &io : imo) {
        addInputToOutput(options, out, io.input, io.mesh.mesh
                         , io.mesh.projected, io.submeshIndex
                         , io.uvAreaScale);
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
                     , bool generateNavtile
                     , const MergeOptions &options
                     , const ExtraOptions &extraOptions)
{
    Output result(tileId, input, navtileSource);
    if (input.tileId().lod == tileId.lod) {
        // as is -> copy
        result.mesh = cloneEntity(input.mesh());
        result.geomExtents = input.node().geomExtents;

        // update surface references of all submeshes
        for (auto &sm : *result.mesh) {
            sm.surfaceReference = input.id() + 1;
        }

        if (input.hasAtlas()) {
            result.atlas = cloneEntity(input.atlas());
        }
        if (input.hasNavtile()) {
            result.navtile = cloneEntity(input.navtile());
        }
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

    CsConvertor phys2sd;
    if (!extraOptions.meshesInSds) {
        phys2sd = CsConvertor(nodeInfo.referenceFrame().model.physicalSrs
                              , nodeInfo.srs());
    }

    const auto coverageVertices
        (inputCoverageVertices
         (input, nodeInfo, phys2sd, options.safetyMargin));
    SdMeshConvertor sdmc(nodeInfo, options.safetyMargin, localId
                         , extraOptions.meshesInSds);

    std::size_t smIndex(0);
    for (const auto &sm : input.mesh()) {
        auto clipped(clip(sm, coverageVertices[smIndex]
                          , coverageExtents(options.safetyMargin), sdmc));

        if (clipped) {
            addInputToOutput(options, result, input, clipped.mesh
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
    SurrogateCalculator() : sum_(), weightSum_() {}

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

    /** Updates surrogate calculation with last computed mesh
     *
     * \param imo generated intermediate output, last submesh is used
     * \param input input used to generate mesh, metanode is used
     * \param source source submesh from input
     */
    void update(const IntermediateOutput::list &imo, const SubMesh &source)
    {
        // grab last input
        if (imo.empty()) { return; }
        auto &im(imo.back());

        // get submesh
        const auto &submesh(im.mesh.mesh);
        if (submesh.empty()) { return; }

        // compute contribution
        const auto weight(area3d(submesh) / area3d(source));
        sum_ += weight * im.input.get().node().geomExtents.surrogate;
        weightSum_ += weight;
    }

    double surrogate() const {
        return sum_ / weightSum_;
    }

private:
    double sum_;
    double weightSum_;
};

inline math::Point3d skirtVector(const math::Point3d &p, float z
                                 , const MergeOptions &options)
{
    return { 0.0, 0.0, (z - p(2)) * options.skirtScale };
}

math::Point2i clip(const Coverage &coverage, const math::Point3d &p)
{
    math::Point2i pp(std::floor(p(0)), std::floor(p(1)));
    if (pp(0) < 0) { pp(0) = 0; }
    if (pp(0) >= coverage.hm.cols) { pp(0) = coverage.hm.cols - 1; }

    if (pp(1) < 0) { pp(1) = 0; }
    if (pp(1) >= coverage.hm.rows) { pp(1) = coverage.hm.rows - 1; }

    return pp;
}

SkirtVectorCallback skirtVectorCallback(const Input &input
                                        , const Coverage &coverage
                                        , const MergeOptions &options)
{
    const auto &cookieCutter(coverage.cookieCutters[input.id()]);

    switch (options.skirtMode) {
    case SkirtMode::none:
        return [](const math::Point3d&) { return math::Point3d(); };

    case SkirtMode::minimum:
        return [&](const math::Point3d &p) -> math::Point3d
        {
            const auto cp(clip(coverage, p));
            if (!cookieCutter.border.get(cp(0), cp(1))) { return {}; }

            if (auto z = coverage.hmMin(cp(0), cp(1))) {
                return skirtVector(p, *z, options);
            }
            return {};
        };

    case SkirtMode::maximum:
        return [&](const math::Point3d &p) -> math::Point3d
        {
            const auto cp(clip(coverage, p));
            if (!cookieCutter.border.get(cp(0), cp(1))) { return {}; }

            if (auto z = coverage.hmMax(cp(0), cp(1))) {
                return skirtVector(p, *z, options);
            }
            return {};
        };

    case SkirtMode::average:
        return [&](const math::Point3d &p) -> math::Point3d
        {
            const auto cp(clip(coverage, p));
            if (!cookieCutter.border.get(cp(0), cp(1))) { return {}; }

            if (auto z = coverage.hmAvg(cp(0), cp(1))) {
                return skirtVector(p, *z, options);
            }
            return {};
        };
    }

    LOGTHROW(err2, std::logic_error)
        << "Skirt mode has invalid value "
        << static_cast<int>(options.skirtMode)
        << "; looks like an uninitialized value, check your code.";
    throw;
}

inline bool needCookieCutters(const MergeOptions &options)
{
    return ((options.glueMode == GlueMode::coverageContour)
            || (options.skirtMode != SkirtMode::none));
}

} // namespace

Output mergeTile(const TileId &tileId
                 , const NodeInfo &nodeInfo
                 , const Input::list &currentSource
                 , const TileSource &parentSource
                 , const MergeConstraints &constraints
                 , const MergeOptions &options
                 , const ExtraOptions &extraOptions)
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

    if ((source.size() == 1) && !extraOptions.checkCoverageForSingleSources) {
        Output result(tileId, source, navtileSource);
        if (!constraints.feasible(result)) { return result; }

        // just one source
        return singleSourced(tileId, nodeInfo, source.front()
                             , filterSources(source, navtileSource)
                             , constraints.generateNavtile()
                             , options, extraOptions);
    }

    // merge result
    Output result(tileId);

    // analyze coverage
    Coverage coverage(tileId, nodeInfo, source, options
                      , needCookieCutters(options));

    // various debug dumps follow

    if (MERGE_MASK_DUMP_DIR) {
        coverage.dump(MERGE_MASK_DUMP_DIR);
    }

    if (MERGE_COOKIE_CUTTERS_DUMP_DIR) {
        coverage.dumpCookieCutters(MERGE_COOKIE_CUTTERS_DUMP_DIR);
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
                             , constraints.generateNavtile()
                             , options, extraOptions);
    }

    // merge meshes
    CsConvertor phys2sd;
    if (!extraOptions.meshesInSds) {
        phys2sd = CsConvertor(nodeInfo.referenceFrame().model.physicalSrs
                              , nodeInfo.srs());
    }

    // process all input tiles from result source (i.e. only those contributing
    // to the tile)

    SurrogateCalculator sc;
    IntermediateOutput::list imo;

    // mesh convertor (for this tile)
    SdMeshConvertor thisSdmc(nodeInfo, options.safetyMargin
                             , {}, extraOptions.meshesInSds);

    // process all inputs
    for (const auto &input : result.source.mesh) {
        const auto &mesh(input.mesh());

        // get current mesh vertices converted to coverage coordinate system
        const auto coverageVertices
            (inputCoverageVertices
             (input, nodeInfo, phys2sd, options.safetyMargin));

        auto icoverageVertices(coverageVertices.begin());

        // localize tileId -> used to map fallback content into this tile
        const auto &tileLod(input.tileId().lod);
        const auto localId(local(tileLod, tileId));

        // mesh vertex convertor: use convertor for this tile or create lazy
        // instance if we are using fallback data (deriving subtile)
        auto sdmc(localId.lod
                  ? SdMeshConvertor::Lazy(nodeInfo, options.safetyMargin
                                          , localId, extraOptions.meshesInSds)
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
                sc.update(imo, sm);
            }
        }
    }

    // add if asked to skirt
    if (options.skirtMode != SkirtMode::none) {
        coverage.dilateHm();

        for (auto &io : imo) {
            addSkirt(io.mesh, thisSdmc
                     , skirtVectorCallback(io.input, coverage, options));
        }
    }

    // pass intermediate result to output
    toOutput(options, result, imo);

    // generate navtile if asked to
    if (constraints.generateNavtile()) { mergeNavtile(result); }

    // set surrogate
    result.geomExtents.surrogate = sc.surrogate();

    return result;
}

} } } // namespace vtslibs::vts::merge
