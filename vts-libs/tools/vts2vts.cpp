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
#include <cstdlib>
#include <string>
#include <iostream>
#include <algorithm>
#include <iterator>

#include <boost/algorithm/string/split.hpp>

#include <opencv2/highgui/highgui.hpp>

#include "dbglog/dbglog.hpp"
#include "utility/streams.hpp"

#include "utility/buildsys.hpp"
#include "utility/gccversion.hpp"
#include "utility/progress.hpp"
#include "utility/streams.hpp"
#include "utility/openmp.hpp"
#include "utility/progress.hpp"

#include "service/cmdline.hpp"

#include "math/transform.hpp"
#include "math/filters.hpp"

#include "imgproc/scanconversion.hpp"
#include "imgproc/jpeg.hpp"

#include "geo/csconvertor.hpp"
#include "geo/coordinates.hpp"

#include "../registry/po.hpp"
#include "../vts.hpp"
#include "../vts/encoder.hpp"
#include "../vts/opencv/navtile.hpp"
#include "../vts/io.hpp"
#include "../vts/csconvertor.hpp"
#include "../vts/meshopinput.hpp"
#include "../vts/meshop.hpp"
#include "../vts/heightmap.hpp"


namespace po = boost::program_options;
namespace vs = vtslibs::storage;
namespace vr = vtslibs::registry;
namespace vts = vtslibs::vts;
namespace ba = boost::algorithm;
namespace fs = boost::filesystem;
namespace ublas = boost::numeric::ublas;

namespace {

struct Config {
    std::string referenceFrame;
    boost::optional<std::uint16_t> textureLayer;
    int textureQuality;
    std::set<vts::TileId> debugTileIds;
    bool forceWatertight;
    boost::optional<vts::LodTileRange> tileExtents;
    double clipMargin;
    double borderClipMargin;

    Config()
        : textureQuality(85), forceWatertight(false)
        , clipMargin(1.0 / 128.), borderClipMargin(1.0 / 128.)
    {}

    double maxClipMargin() const {
        // no tile extents: use only clip margin
        if (!tileExtents) { return clipMargin; }
        // use maximum of clipe extents and border clip extents
        return std::max(clipMargin, borderClipMargin);
    }
};

class Vts2Vts : public service::Cmdline
{
public:
    Vts2Vts()
        : service::Cmdline("vts2vts", BUILD_TARGET_VERSION)
        , createMode_(vts::CreateMode::failIfExists)
    {
    }

private:
    virtual void configuration(po::options_description &cmdline
                               , po::options_description &config
                               , po::positional_options_description &pd)
        UTILITY_OVERRIDE;

    virtual void configure(const po::variables_map &vars)
        UTILITY_OVERRIDE;

    virtual bool help(std::ostream &out, const std::string &what) const
        UTILITY_OVERRIDE;

    virtual int run() UTILITY_OVERRIDE;

    fs::path input_;
    fs::path output_;

    vts::CreateMode createMode_;

    Config config_;
};

void Vts2Vts::configuration(po::options_description &cmdline
                             , po::options_description &config
                             , po::positional_options_description &pd)
{
    vr::registryConfiguration(cmdline, vr::defaultPath());

    cmdline.add_options()
        ("input", po::value(&input_)->required()
         , "Path to input (vts) tile set.")
        ("output", po::value(&output_)->required()
         , "Path to output (vts) tile set.")
        ("overwrite", "Existing tile set gets overwritten if set.")

        ("referenceFrame", po::value(&config_.referenceFrame)->required()
         , "Destination reference frame. Must be different from input "
         "tileset's referenceFrame.")

        ("textureQuality", po::value(&config_.textureQuality)
         ->default_value(config_.textureQuality)->required()
         , "Texture quality for JPEG texture encoding (0-100).")

        ("textureLayer", po::value<std::string>()
         , "String/numeric id of bound layer to be used as external texture "
         "in generated meshes.")

        ("force.watertight", po::value(&config_.forceWatertight)
         ->default_value(false)->implicit_value(true)
         , "Enforces full coverage mask to every generated tile even "
         "when it is holey.")

        ("clipMargin", po::value(&config_.clipMargin)
         ->default_value(config_.clipMargin)
         , "Margin (in fraction of tile dimensions) added to tile extents in "
         "all 4 directions.")

        ("tileExtents", po::value<vts::LodTileRange>()
         , "Optional tile extents specidied in form lod/llx,lly:urx,ury. "
         "When set, only tiles in that range and below are added to "
         "the output.")

        ("borderClipMargin", po::value(&config_.borderClipMargin)
         , "Margin (in fraction of tile dimensions) added to tile extents "
         "where tile touches artificial border definied by tileExtents.")

        ("debug.tileId", po::value<std::vector<vts::TileId>>()
         , "Limits output only to given set of tiles. "
         "Used for debugging purposes.")
        ;

    pd.add("input", 1);
    pd.add("output", 1);

    (void) config;
}

void Vts2Vts::configure(const po::variables_map &vars)
{
    vr::registryConfigure(vars);

    createMode_ = (vars.count("overwrite")
                   ? vts::CreateMode::overwrite
                   : vts::CreateMode::failIfExists);

    if (vars.count("textureLayer")) {
        auto value(vars["textureLayer"].as<std::string>());

        vr::BoundLayer layer;
        try {
            layer = vr::system.boundLayers(boost::lexical_cast<int>(value));
        } catch (const boost::bad_lexical_cast&) {
            layer = vr::system.boundLayers(value);
        }

        if (layer.type != vr::BoundLayer::Type::raster) {
            throw po::validation_error
                (po::validation_error::invalid_option_value, "textureLayer");
        }
        config_.textureLayer = layer.numericId;
    }

    if ((config_.textureQuality < 0) || (config_.textureQuality > 100)) {
        throw po::validation_error
            (po::validation_error::invalid_option_value, "textureQuality");
    }

    if (vars.count("tileExtents")) {
        config_.tileExtents = vars["tileExtents"].as<vts::LodTileRange>();
    }

    if (vars.count("debug.tileId")) {
        const auto &debugTileIds(vars["debug.tileId"]
                                 .as<std::vector<vts::TileId>>());
        config_.debugTileIds.insert(debugTileIds.begin(), debugTileIds.end());
    }
}

bool Vts2Vts::help(std::ostream &out, const std::string &what) const
{
    if (what.empty()) {
        out << R"RAW(vts2vts
usage
    vts2vts INPUT OUTPUT [OPTIONS]

)RAW";
    }
    return false;
}

double triangleArea(const math::Point2 &a, const math::Point2 &b,
                    const math::Point2 &c)
{
    return std::abs
        (math::crossProduct(math::Point2(b - a), math::Point2(c - a)))
        / 2.0;
}

double bestTileArea(const math::Points2 &corners)
{
    return (triangleArea(corners[0], corners[1], corners[2])
            + triangleArea(corners[2], corners[3], corners[0]));
}

int bestLod(const vr::ReferenceFrame::Division::Node &node, double area)
{
    // compute longest of base node tile sizes
    auto rootSize(math::size(node.extents));
    auto rootArea(rootSize.width * rootSize.height);

    // compute number of requested tiles per edge
    auto tileCount(std::sqrt(rootArea / area));

    return int(std::round(std::log2(tileCount)));
}

vts::TileRange::point_type
tiled(const math::Size2f &ts, const math::Point2 &origin
      , const math::Point2 &p)
{
    math::Point2 local(p - origin);
    return vts::TileRange::point_type(local(0) / ts.width
                                      , -local(1) / ts.height);
}

vts::TileRange tileRange(const vr::ReferenceFrame::Division::Node &node
                         , vts::Lod localLod, const math::Points2 &points
                         , double margin)
{
    const auto ts(vts::tileSize(node.extents, localLod));
    // NB: origin is in upper-left corner and Y grows down
    const auto origin(math::ul(node.extents));

    math::Size2f isize(ts.width * margin, ts.height * margin);
    std::array<math::Point2, 4> inflates{{
            { -isize.width, +isize.height }
            , { +isize.width, +isize.height }
            , { +isize.width, -isize.height }
            , { -isize.width, -isize.height }
        }};

    vts::TileRange r(math::InvalidExtents{});

    for (const auto &p : points) {
        for (const auto &inflate : inflates) {
            update(r, tiled(ts, origin, p + inflate));
        }
    }

    // globalize tile range
    return vts::global(node.id, localLod, r);
}

template <typename Op>
void forEachTile(const vr::ReferenceFrame &referenceFrame
                 , vts::Lod lod, const vts::TileRange &tileRange
                 , Op op)
{
    typedef vts::TileRange::value_type Index;
    for (Index j(tileRange.ll(1)), je(tileRange.ur(1)); j <= je; ++j) {
        for (Index i(tileRange.ll(0)), ie(tileRange.ur(0)); i <= ie; ++i) {
            op(vts::NodeInfo(referenceFrame, vts::TileId(lod, i, j)));
        }
    }
}

template <typename Op>
void rasterizeTiles(const vr::ReferenceFrame &referenceFrame
                    , const vr::ReferenceFrame::Division::Node &rootNode
                    , vts::Lod lod, const vts::TileRange &tileRange
                    , Op op)
{
    // process tile range
    forEachTile(referenceFrame, lod, tileRange
                , [&](const vts::NodeInfo &ni)
    {
        if (!ni.productive()) { return; }

        LOG(info1)
            << std::fixed << "dst tile: "
            << ni.nodeId() << ", " << ni.extents();

        // TODO: check for incidence with Q; NB: clip margin must be taken into
        // account

        // check for root
        if (ni.subtree().root().id == rootNode.id) {
            op(vts::tileId(ni.nodeId()));
        }
    });
}

typedef std::map<vts::TileId, vts::TileId::list> SourceInfo;

math::Points2 projectCorners(const vr::ReferenceFrame::Division::Node &node
                             , const vts::CsConvertor &conv
                             , const math::Points2 &src)
{
    math::Points2 dst;
    try {
        for (const auto &c : src) {
            dst.push_back(conv(c));
            LOG(info1) << std::fixed << "corner: "
                       << c << " -> " << dst.back();
            if (!inside(node.extents, dst.back())) {
                // projected dst tile cannot fit inside this node's
                // extents -> ignore
                return {};
            }
        }
    } catch (const std::exception&) {
        // whole tile cannot be projected -> ignore
        return {};
    }

    // OK, we could convert whole tile into this reference system
    return dst;
}

class SourceInfoBuilder : boost::noncopyable {
public:
    SourceInfoBuilder(const vts::TileSet &tileset
                      , const vr::ReferenceFrame &dstRf
                      , double margin)
    {
        const auto &srcRf(tileset.referenceFrame());
        utility::Progress progress(tileset.tileIndex().count());

        traverse(tileset.tileIndex()
                 , [&](const vts::TileId &srcId, vts::QTree::value_type flags)
        {
            (++progress).report
                (utility::Progress::ratio_t(5, 1000)
                 , "building tile mapping ");
            if (!vts::TileIndex::Flag::isReal(flags)) { return; }

            vts::NodeInfo ni(srcRf, srcId);
            if (!ni.valid()) { return; }

            const auto &srcExtents(ni.extents());
            const math::Points2 srcCorners = {
                ul(srcExtents), ur(srcExtents), lr(srcExtents), ll(srcExtents)
            };

            // for each destination node
            for (const auto &item : dstRf.division.nodes) {
                const auto &node(item.second);
                if (!node.real()) { continue; }
                const vts::CsConvertor csconv(ni.srs(), node.srs);

                auto dstCorners(projectCorners(node, csconv, srcCorners));

                // ignore tiles that cannot be transformed
                if (dstCorners.empty()) { continue; }

                // find best tile size
                auto bta(bestTileArea(dstCorners));

                // find such the closest tile to the best tile size
                auto dstLocalLod(bestLod(node, bta));
                auto dstLod(node.id.lod + dstLocalLod);

                LOG(info1)
                    << "Best tile area: " << bta << " -> LOD: " << dstLod
                    << " (node's local LOD: " << dstLocalLod << ").";

                // generate tile range from corners
                auto tr(tileRange(node, dstLocalLod, dstCorners, margin));
                LOG(info1) << "tile range: " << tr;

                // TODO: add margin
                rasterizeTiles(dstRf, node, dstLod, tr
                               , [&](const vts::TileId &id)
                {
                    sourceInfo_[id].push_back(srcId);
                    dstTi_.set(id, 1);
                });
            }
        });

        // clone dst tile index to valid tree and make it complete
        validTree_ = vts::TileIndex
            (vts::LodRange(0, dstTi_.maxLod()), &dstTi_);
        validTree_.complete();
    }

    const vts::TileIndex* validTree() const { return &validTree_; }

    const vts::TileId::list& source(const vts::TileId &tileId) const {
        auto fsourceInfo(sourceInfo_.find(tileId));
        if (fsourceInfo == sourceInfo_.end()) { return emptySource_; }
        return fsourceInfo->second;
    }

    std::size_t size() const { return sourceInfo_.size(); }

private:
    SourceInfo sourceInfo_;
    vts::TileIndex dstTi_;
    vts::TileIndex validTree_;

    static const vts::TileId::list emptySource_;
};

// keep empty, used as placeholder!
const vts::TileId::list SourceInfoBuilder::emptySource_;

class Encoder : public vts::Encoder {
public:
    Encoder(const boost::filesystem::path &path
            , const vts::TileSetProperties &properties
            , vts::CreateMode mode
            , const vts::TileSet &input
            , const Config &config)
        : vts::Encoder(path, properties, mode)
        , config_(config), input_(input)
        , inputSource_(tilesetDataSource(input_))
        , srcRf_(input_.referenceFrame())
        , srcInfo_(input_, referenceFrame(), config.maxClipMargin())
        , debugTileIds_(config.debugTileIds.empty() ? nullptr
                        : &config.debugTileIds)
    {
        setConstraints(Constraints().setValidTree(srcInfo_.validTree()));
        setEstimatedTileCount(srcInfo_.size());
    }

private:
    virtual TileResult
    generate(const vts::TileId &tileId, const vts::NodeInfo &nodeInfo
             , const TileResult&)
        UTILITY_OVERRIDE;

    virtual void finish(vts::TileSet&);

    const Config config_;

    const vts::TileSet &input_;
    const vts::MeshOpInput::DataSource::pointer inputSource_;
    const vr::ReferenceFrame srcRf_;

    SourceInfoBuilder srcInfo_;

    const std::set<vts::TileId> *debugTileIds_;
};

void warpInPlace(const vts::CsConvertor &conv, vts::SubMesh &sm)
{
    // just convert vertices
    for (auto &v : sm.vertices) {
        // convert vertex in-place
        v = conv(v);
    }
}

vts::VertexMask warpInPlaceWithMask(vts::GeomExtents &ge
                                    , const vts::CsConvertor &conv
                                    , vts::SubMesh &sm)
{
    vts::VertexMask mask(sm.vertices.size(), true);

    std::size_t masked(0);
    auto imask(mask.begin());
    for (auto &v : sm.vertices) {
        try {
            // convert vertex in-place
            v = conv(v);
            update(ge, v(2));
        } catch (const std::exception&) {
            // cannot convert vertex -> mask out
            *imask = false;
            ++masked;
        }
        ++imask;
    }

    // nothing masked -> no mask
    if (!masked) { return {}; }

    // something masked
    return mask;
}

math::Size2 navpaneSizeInPixels(const math::Size2 &sizeInTiles)
{
    // NB: navtile is in grid system, border pixels are shared between adjacent
    // tiles
    auto s(vts::NavTile::size());
    return { 1 + sizeInTiles.width * (s.width - 1)
            , 1 + sizeInTiles.height * (s.height - 1) };
}

vts::NavTile::pointer
warpNavtiles(const vts::TileId &tileId
             , const vr::ReferenceFrame &referenceFrame
             , const vts::NodeInfo &nodeInfo
             , const vts::MeshOpInput::list &source)
{
    // TODO: Check for different lodding/SDS and process accordingly

    vts::HeightMap hm(tileId, source, referenceFrame);
    if (hm.empty()) { return {}; }
    hm.warp(nodeInfo);

    auto navtile(hm.navtile(tileId));
    if (navtile->coverageMask().empty()) { return {}; }

    return navtile;
}

class SurrogateCalculator {
public:
    SurrogateCalculator()
        : sum_(), weightSum_()
    {}

    /** Updates surrogate from clipped mesh
     */
    void update(const vts::SubMesh &src, vts::SubMesh &dst
                , const vts::VertexMask &srcMask
                , float srcSurrogate)
    {
        if (!vts::GeomExtents::validSurrogate(srcSurrogate)) {
            return;
        }

        const auto srcArea(srcMask.empty()
                           ? area3d(src) : area3d(src, srcMask));
        const auto weight(area3d(dst) / srcArea);

        sum_ += weight * srcSurrogate;
        weightSum_ += weight;
    }

    float surrogate() const {
        if (weightSum_) { return sum_ / weightSum_; }
        return vts::GeomExtents::invalidSurrogate;
    }

private:
    double sum_;
    double weightSum_;
};

inline float getInputSurrogate(const vr::ReferenceFrame &inputRf
                               , const vts::MeshOpInput &input
                               , const std::string &sds)
{
    // convert surrogate from source SDS to destination SDS

    const auto &ge(input.node().geomExtents);
    if (!ge.validSurrogate()) { return vts::GeomExtents::invalidSurrogate; }

    const vts::NodeInfo ni(inputRf, input.tileId());

    const vts::CsConvertor conv(ni.srs(), sds);

    const auto &extents(ni.extents());
    const auto center(math::center(extents));

    auto convert([&](const math::Point2 &p) -> boost::optional<double>
    {
        try {
            return conv(math::Point3(p(0), p(1), ge.surrogate))(2);
        } catch (const geo::ProjectionError&) {}
        return boost::none;
    });

    // try center
    if (auto h = convert(center)) { return *h; }
    // try extents corners
    for (const auto &p : math::vertices(extents)) {
        if (auto h = convert(p)) { return *h; }
    }

    // TODO: check centers of extent edges

    // nothing matched
    return vts::GeomExtents::invalidSurrogate;
}

Encoder::TileResult
Encoder::generate(const vts::TileId &tileId, const vts::NodeInfo &nodeInfo
                  , const TileResult&)
{
    // check for debug
    if (debugTileIds_ && !debugTileIds_->count(tileId)) {
        return TileResult::Result::noDataYet;
    }

    vts::BorderCondition borderCondition;

    if (config_.tileExtents) {
        borderCondition = vts::inside(*config_.tileExtents, tileId);
        if (!borderCondition) {
            // outside of range
            return TileResult::Result::noDataYet;
        }
    }

    const auto &src(srcInfo_.source(tileId));
    if (src.empty()) {
        return TileResult::Result::noDataYet;
    }

    LOG(info1) << "Source tiles(" << src.size() << "): "
               << utility::join(src, ", ") << ".";

    vts::MeshOpInput::list source;
    {
        vts::MeshOpInput::Id id(0);
        for (const auto &srcId : src) {
            UTILITY_OMP(critical)
            {
                // build input for tile transformation:
                //     * node info is generated on the fly
                //     * this cannot be a lazy operation
                vts::MeshOpInput t(id++, inputSource_, srcId, nullptr, false);
                if (t) { source.push_back(t); }
            }
        }
    }

    // CS convertors
    // src physical -> dst SDS
    const vts::CsConvertor srcPhy2Sds
        (srcRf_.model.physicalSrs, nodeInfo.srs());

    // dst SDS -> dst physical
    const vts::CsConvertor sds2DstPhy
        (nodeInfo.srs(), referenceFrame().model.physicalSrs);

    auto clipExtents(vts::inflateTileExtents
                     (nodeInfo.extents(), config_.clipMargin
                      , borderCondition, config_.borderClipMargin));

    // output
    Encoder::TileResult result;
    auto &tile(result.tile());
    vts::Mesh &mesh
        (*(tile.mesh = std::make_shared<vts::Mesh>(config_.forceWatertight)));
    vts::RawAtlas::pointer patlas([&]() -> vts::RawAtlas::pointer
    {
        auto atlas(std::make_shared<vts::RawAtlas>());
        tile.atlas = atlas;
        return atlas;
    }());
    auto &atlas(*patlas);

    SurrogateCalculator sc;

    for (const auto &input : source) {
        const auto &inMesh(input.mesh());
        const auto srcSurrogate
            (getInputSurrogate(srcRf_, input, nodeInfo.srs()));

        for (std::size_t smIndex(0), esmIndex(inMesh.size());
             smIndex != esmIndex; ++smIndex)
        {
            auto sm(inMesh[smIndex]);

            auto mask(warpInPlaceWithMask(tile.geomExtents, srcPhy2Sds, sm));

            // clip submesh
            auto dstSm(vts::clip(sm, clipExtents, mask));

            if (dstSm.empty()) { continue; }

            // valid mesh

            // update surrogate
            sc.update(sm, dstSm, mask, srcSurrogate);

            // re-generate external tx coordinates (if division node allows)
            generateEtc(dstSm, nodeInfo.extents()
                        , nodeInfo.node().externalTexture);

            // update mesh coverage mask
            if (!config_.forceWatertight) {
                updateCoverage(mesh, dstSm, nodeInfo.extents());
            }

            // set new texture layer if provided
            dstSm.textureLayer = config_.textureLayer;

            // convert mesh to destination physical SRS
            warpInPlace(sds2DstPhy, dstSm);

            // add mesh
            mesh.add(dstSm);

            // copy texture if submesh has atlas
            if (input.hasAtlas() && input.atlas().valid(smIndex)) {
                atlas.add(input.atlas().get(smIndex));
            }

            // update credits
            const auto &credits(input.node().credits());
            tile.credits.insert(credits.begin(), credits.end());
        }
    }

    if (mesh.empty()) {
       // no mesh
        // decrement number of estimated tiles
        updateEstimatedTileCount(-1);
        // tell that there is nothing yet
        return TileResult::Result::noDataYet;
    }

    // warp navtile and its mask
    tile.navtile = warpNavtiles(tileId, srcRf_, nodeInfo, source);

    // merge submeshes if allowed
    std::tie(tile.mesh, tile.atlas)
        = mergeSubmeshes(tileId, tile.mesh, patlas, config_.textureQuality);

    if (atlas.empty()) {
        // no atlas -> disable
        tile.atlas.reset();
    }

    // set surrogate (z is already covered)
    tile.geomExtents.surrogate = sc.surrogate();

    // done:
    return result;
}

void Encoder::finish(vts::TileSet &ts)
{
    auto position(input_.getProperties().position);

    // convert initial position -- should work
    const vts::CsConvertor nav2nav(input_.referenceFrame().model.navigationSrs
                                   , referenceFrame().model.navigationSrs);
    position.position = nav2nav(position.position);

    // store
    ts.setPosition(position);
}

int Vts2Vts::run()
{
    auto input(vts::openTileSet(input_));

    // clone info from input tileset
    auto oldProperties(input.getProperties());
    vts::TileSetProperties properties;
    properties.referenceFrame = config_.referenceFrame;
    properties.id = oldProperties.id;
    properties.credits = oldProperties.credits;

    // TODO: bound layers

    // run the encoder
    Encoder(output_, properties, createMode_, input, config_).run();

    // all done
    LOG(info4) << "All done.";
    return EXIT_SUCCESS;
}

} // namespace

int main(int argc, char *argv[])
{
    return Vts2Vts()(argc, argv);
}
