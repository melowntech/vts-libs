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
#include <map>
#include <fstream>

#include "math/transform.hpp"

#include "utility/streams.hpp"
#include "utility/openmp.hpp"

#include "imgproc/scanconversion.hpp"

#include "heightmap.hpp"
#include "ntgenerator.hpp"

namespace fs = boost::filesystem;

namespace vtslibs { namespace vts {

namespace {

std::map<std::string, const RFNode*>
sds2rfnode(const registry::ReferenceFrame &referenceFrame)
{
    std::map<std::string, const RFNode*> map;
    for (const auto &node : vts::NodeInfo::nodes(referenceFrame)) {
        map.insert(std::map<std::string, const RFNode*>::value_type
                   (node.srs(), &node.subtree().root()));
    }
    return map;
}

struct NavtileInfo {
    LodRange lodRange;
    double pixelSize;

    NavtileInfo(const LodRange &lodRange, double pixelSize)
        : lodRange(lodRange), pixelSize(pixelSize)
    {}

    typedef std::map<const RFNode*, NavtileInfo> map;
};

} // namespace

struct NtGenerator::Accumulator : boost::static_visitor<CsConvertor> {
    typedef std::shared_ptr<Accumulator> pointer;
    typedef std::map<const RFNode*, pointer> map;

    NavtileInfo ntInfo;
    HeightMap::Accumulator hma;
    std::string sds;
    std::string navigationSrs;

    Accumulator(const registry::ReferenceFrame &rf, const RFNode &node
                  , const NavtileInfo &ntInfo)
        : ntInfo(ntInfo), hma(ntInfo.lodRange.max)
        , sds(node.srs), navigationSrs(rf.model.navigationSrs)
    {}

    CsConvertor toNavSrs(const TileSrs &srs) const {
        return boost::apply_visitor(*this, srs);
    }

    CsConvertor operator()(const boost::blank&) const {
        return CsConvertor(sds, navigationSrs);
    }

    CsConvertor operator()(const std::string &srs) const {
        return CsConvertor(srs, navigationSrs);
    }

    CsConvertor operator()(const geo::SrsDefinition &srs) const {
        return CsConvertor(srs, navigationSrs);
    }
};

NtGenerator::NtGenerator(const registry::ReferenceFrame *referenceFrame
                         , const boost::optional<fs::path> &path)
    : referenceFrame_(referenceFrame)
    , sds2rfnode_(sds2rfnode(*referenceFrame_))
{
    if (path) { load(*path); }
}

void NtGenerator::addAccumulator(const std::string &sds
                                 , const LodRange &lodRange
                                 , double pixelSize)
{
    auto fsds2rfnode(sds2rfnode_.find(sds));
    if (fsds2rfnode == sds2rfnode_.end()) {
        LOGTHROW(err2, std::runtime_error)
            << "Cannot find node by srs <" << sds << ">.";
    }

    auto *node(fsds2rfnode->second);

    accumulators_.insert
        (Accumulator::map::value_type
         (node, std::make_shared<Accumulator>
          (*referenceFrame_, *node
           , NavtileInfo(lodRange, pixelSize))));
}

void NtGenerator::load(const fs::path &path)
{
    std::ifstream f(path.string());

    std::string sds;
    LodRange lodRange;
    double pixelSize;
    while (f >> sds >> lodRange >> pixelSize) {
        addAccumulator(sds, lodRange, pixelSize);
    }

    if (!f.eof()) {
        LOGTHROW(err2, std::runtime_error)
            << "Unable to load navtile info from " << path << ".";
    }

    f.close();
}

void NtGenerator::save(const fs::path &path)
{
    utility::ofstreambuf f;
    f.exceptions(std::ofstream::failbit | std::ofstream::badbit);
    f.open(path.string(), std::ofstream::out | std::ofstream::trunc);

    for (const auto &item : accumulators_) {
        f << item.first->srs << " " << item.second->ntInfo.lodRange
          << std::fixed << std::setprecision(9)
          << " " << item.second->ntInfo.pixelSize << '\n';
    }

    f.close();
}

namespace {

template <typename Op>
void rasterizeMesh(const vts::Mesh &mesh, const vts::CsConvertor &toNavSrs
                   , const math::Matrix4 &trafo
                   , const math::Size2 &rasterSize, Op op)
{
    math::Points3 vertices;
    std::vector<imgproc::Scanline> scanlines;
    cv::Point3f tri[3];

    for (const auto &sm : mesh) {
        vertices.reserve(sm.vertices.size());
        vertices.clear();
        for (auto v : sm.vertices) {
            v(2) = toNavSrs(v)(2);
            vertices.push_back(transform(trafo, v));
        }

        for (const auto &face : sm.faces) {
            for (int i(0); i < 3; ++i) {
                const auto &p(vertices[face[i]]);
                tri[i].x = p(0);
                tri[i].y = p(1);
                tri[i].z = p(2);
            }

            scanlines.clear();
            imgproc::scanConvertTriangle(tri, 0, rasterSize.height, scanlines);

            for (const auto &sl : scanlines) {
                imgproc::processScanline(sl, 0, rasterSize.width, op);
            }
        }
    }
}

inline math::Matrix4 mesh2grid(const math::Extents2 &extents
                              , const math::Size2 &gridSize)
{
    math::Matrix4 trafo(ublas::identity_matrix<double>(4));

    auto es(size(extents));

    // scales
    math::Size2f scale((gridSize.width - 1) / es.width
                       , (gridSize.height - 1) / es.height);

    // scale to grid
    trafo(0, 0) = scale.width;
    trafo(1, 1) = -scale.height;

    // place zero to upper-left corner
    trafo(0, 3) = -extents.ll(0) * scale.width;
    trafo(1, 3) = extents.ur(1) * scale.height;

    return trafo;
}

} // namespace

void NtGenerator::addTile(const TileId &tileId, const NodeInfo &nodeInfo
                          , const Mesh &mesh, const TileSrs &srs)
{
    // grab accumulator
    auto faccumulators(accumulators_.find(&nodeInfo.subtree().root()));
    if (faccumulators == accumulators_.end()) { return; }
    auto &acc(*faccumulators->second);
    if (tileId.lod != acc.ntInfo.lodRange.max) { return; }

    auto &hm([&]() -> cv::Mat&
    {
        cv::Mat *hm(nullptr);
        UTILITY_OMP(critical(getnavtile))
            hm = &acc.hma.tile(tileId);
        return *hm;
    }());

    // invalid heightmap value (i.e. initial value) is +oo and we take minimum
    // of all rasterized heights in given place
    rasterizeMesh(mesh, acc.toNavSrs(srs)
                  , mesh2grid(nodeInfo.extents(), acc.hma.tileSize())
                  , acc.hma.tileSize(), [&](int x, int y, float z)
    {
        auto &value(hm.at<float>(y, x));
        if (z > value) { value = z; }
    });
}

    void addTile(const TileId &tileId, const NodeInfo &nodeInfo
                 , const Mesh &mesh, const boost::optional<std::string>
                 &srs = boost::none);

namespace {

void fillSurrogate(vts::TileSet &ts, const vts::TileIndex &ti
                   , const vts::HeightMap &hm
                   , const vts::NodeInfo &root, vts::Lod lod
                   , const vts::TileRange &tileRange
                   , NtGenerator::Reporter &reporter)
{
    LOG(info3) << "Computing surrogates at lod " << lod << " from "
               << "heightmap " << hm.size() << ".";

    const vts::CsConvertor navsds2sds(root.navsds(), root.srs());

    traverse(ti, lod
             , [&](const vts::TileId &tileId
                   , vts::QTree::value_type flags)
    {
        // process only tiles with mesh
        if (!(flags & vts::TileIndex::Flag::mesh)) { return; }
        if (!inside(tileRange, vts::point(tileId))) { return; }

        const auto ni(root.child(tileId));
        const auto center(math::center(ni.extents()));
        if (auto height = hm.reconstruct(center)) {
            ts.setSurrogateValue
                (tileId
                 , navsds2sds(math::Point3(center(0), center(1), *height))[2]);
            reporter.report();
        }
    });
}

} // namespace

void NtGenerator::generate(TileSet &ts, double dtmExtractionRadius) const
{
    Reporter dummy;
    generate(ts, dtmExtractionRadius, dummy);
}

void NtGenerator::generate(vts::TileSet &ts, double dtmExtractionRadius
                           , Reporter &reporter)
    const
{
    if (accumulators_.empty()) {
        // we need to report empty set
        reporter.expect(0);
        return;
    }

    boost::optional<vts::HeightMap::BestPosition> bestPosition;
    const auto &navigationSrs(referenceFrame_->model.navigationSrs);

    const auto &ti(ts.tileIndex());
    const auto lodRange(ti.lodRange());

    reporter.expect(ti.count());

    for (auto &item : accumulators_) {
        const auto &rfnode(*item.first);
        const vts::NodeInfo ni(*referenceFrame_, rfnode.id);
        auto &acc(*item.second);

        vts::HeightMap hm
            (std::move(acc.hma), *referenceFrame_
             , dtmExtractionRadius / acc.ntInfo.pixelSize);

        // use best position if better than previous
        {
            auto bp(hm.bestPosition());
            if (!bestPosition
                || (bp.verticalExtent > bestPosition->verticalExtent))
            {
                bp.location
                    = vts::CsConvertor(rfnode.srs, navigationSrs)
                    (bp.location);
                bestPosition = bp;
            }
        }

        const auto &lr(acc.ntInfo.lodRange);

        // limit of navtile's influence
        const vts::Lod nt2Tilelimit(lr.max + vts::NavTile::binOrder);
        // prefill by defaults
        vts::Lod tileLod(lodRange.max);
        auto tileRange(vts::childRange(rfnode.id, tileLod));

        if (tileLod > nt2Tilelimit) {
            // there are some tiles deeper under then navtile influence; let's
            // generate their surrogates from bottom navtiles

            for (; tileLod > nt2Tilelimit;
                 --tileLod, tileRange = vts::parentRange(tileRange))
            {
                fillSurrogate(ts, ti, hm, ni, tileLod, tileRange, reporter);
            }
        } else {
            // data under navtile's influence, peg to bottom navtile lod
            tileLod = nt2Tilelimit;
            tileRange = vts::childRange(rfnode.id, tileLod);
        }

        LOG(info3) << "Extracting navtiles.";

        // iterate in nt lod range backwards: iterate from start and invert
        // forward lod into backward lod
        for (auto lod(lr.max); lod >= lr.min; --lod, --tileLod
                 , tileRange = vts::parentRange(tileRange))
        {
            LOG(info3) << "Setting navtiles at lod " << lod << ".";

            // resize heightmap for given lod
            hm.resize(lod);

            // generate and store navtiles
            // FIXME: traverse only part covered by current node
            traverse(ti, lod
                     , [&](const vts::TileId &tileId
                           , vts::QTree::value_type flags)
            {
                // process only tiles with mesh
                if (!(flags & vts::TileIndex::Flag::mesh)) { return; }

                if (auto nt = hm.navtile(tileId)) {
                    ts.setNavTile(tileId, *nt);
                }
            });

            if (storage::in(tileLod , lodRange)) {
                // tile lod is valid, fill surrogates
                fillSurrogate(ts, ti, hm, ni, tileLod, tileRange, reporter);
            }
        }

        LOG(info3) << "Navtiles extracted.";

        // force halve that is not covered by resize in main loop
        if (tileLod >= lodRange.min) {
            hm.halve();
        }

        // wind up to first valid lod if not there yet
        while (tileLod > lodRange.max) {
            // halve heightmap
            hm.halve();
            --tileLod;
        }

        // process tail
        for (; in(tileLod, lodRange);
             --tileLod, tileRange = vts::parentRange(tileRange))
        {
            // tile lod is valid, fill surrogates
            fillSurrogate(ts, ti, hm, ni, tileLod, tileRange, reporter);

            // halve heightmap
            hm.halve();
        }
    }

    // use best position if available
    if (bestPosition) {
        registry::Position pos;
        pos.position = bestPosition->location;

        pos.type = registry::Position::Type::objective;
        pos.heightMode = registry::Position::HeightMode::fixed;
        pos.orientation = { 0.0, -90.0, 0.0 };
        pos.verticalExtent = bestPosition->verticalExtent;
        pos.verticalFov = registry::Position::naturalFov();
        ts.setPosition(pos);
    }
}

} } // namespace vtslibs::vts
