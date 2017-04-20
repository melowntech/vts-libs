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

#include "dbglog/dbglog.hpp"
#include "utility/streams.hpp"

#include "utility/gccversion.hpp"
#include "utility/progress.hpp"
#include "utility/streams.hpp"

#include "service/cmdline.hpp"

#include "../tilestorage.hpp"
#include "../tilestorage/io.hpp"
#include "../tilestorage/po.hpp"
#include "../tilestorage/tileset-advanced.hpp"

#include "../registry.hpp"
#include "../vts0.hpp"
#include "../vts0/io.hpp"
#include "../tilestorage/po.hpp"
#include "../vts0/tileset-advanced.hpp"

#include "../registry/po.hpp"

namespace po = boost::program_options;
namespace ts = vtslibs::tilestorage;
namespace vts = vtslibs::vts0;
namespace vr = vtslibs::registry;
namespace fs = boost::filesystem;

namespace {

struct FilterSettings {
    boost::optional<ts::Extents> extents;
    boost::optional<vts::LodRange> lodRange;
    boost::optional<fs::path> mask;
};

class TileSet2Vts : public service::Cmdline
{
public:
    TileSet2Vts()
        : service::Cmdline("ts2vts", BUILD_TARGET_VERSION)
        , createMode_(vts::CreateMode::failIfExists)
        , rootDelta_(0.0,0.0)
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

    vts::CreateMode createMode_;
    ts::Locator input_;
    fs::path output_;
    std::string referenceFrame_;
    math::Point2 rootDelta_;
};

void TileSet2Vts::configuration(po::options_description &cmdline
                                , po::options_description &config
                                , po::positional_options_description &pd)
{
    vr::registryConfiguration(cmdline, vr::defaultPath());

    cmdline.add_options()
        ("input", po::value(&input_)->required()
         , "Locator of input old tile set.")
        ("output", po::value(&output_)->required()
         , "Path of output vts tile set.")
        ("overwrite", "Overwrite existing dataset.")
        ("referenceFrame", po::value(&referenceFrame_)->required()
         , "Reference frame.")
        ;

    pd.add("input", 1);
    pd.add("output", 1);

    (void) config;
}

void TileSet2Vts::configure(const po::variables_map &vars)
{
    vr::registryConfigure(vars);

    createMode_ = (vars.count("overwrite")
                   ? vts::CreateMode::overwrite
                   : vts::CreateMode::failIfExists);
}

bool TileSet2Vts::help(std::ostream &out, const std::string &what) const
{
    if (what.empty()) {
        out << R"RAW(ts2vts conversion tool
usage
    ts2vts INPUT OUTPUT [OPTIONS]

This command converts an existing old tileset to new vts format.
)RAW";
    }
    return false;
}

inline vts::TileId asVts(const ts::Index &i)
{
    // since alignment was switched to upper-left corner we have to negate
    // northing; original TileId is tile's lower-left corner therefore we must
    // move to its upper-left corner by subtracting 1
    return { i.lod, i.easting, -1 - i.northing };
}

inline vts::TileId asVts(const ts::Alignment &alignment, long baseTileSize
                         , const ts::TileId &tid)
{
    const auto tileId(asVts(tileIndex(alignment, baseTileSize, tid)));

    if ((tileId.x >> tileId.lod) || (tileId.y >> tileId.lod)) {
        LOGTHROW(err3, std::runtime_error)
            << "TS tile " << tid << " is outside of reference frame extents.";
    }

    return tileId;
}

inline vts::TileFile asVts(const ts::TileFile &f)
{
    switch (f) {
    case ts::TileFile::mesh: return vts::TileFile::mesh;
    case ts::TileFile::atlas: return vts::TileFile::atlas;
    case ts::TileFile::meta: return vts::TileFile::meta;
    default: throw "Unexpected TileFile value. Go fix your program.";
    }
    throw;
}

vts::MetaNode asVts(const ts::MetaNode &src)
{
    vts::MetaNode dst;
    dst.zmin = src.zmin;
    dst.zmax = src.zmax;
    dst.gsd = src.gsd;
    dst.coarseness = src.coarseness;

    std::copy(&src.heightmap[0][0]
              , &src.heightmap[ts::MetaNode::HMSize - 1][ts::MetaNode::HMSize]
              , &dst.heightmap[0][0]);
    std::copy(&src.pixelSize[0][0]
              , &src.pixelSize[1][2]
              , &dst.pixelSize[0][0]);
    return dst;
}

vts::LodLevels asVts(const ts::LodLevels &src)
{
    vts::LodLevels dst;
    dst.lod = src.lod;
    dst.delta = src.delta;
    return dst;
}

ts::TileId applyDelta(const ts::TileId &tileId, const math::Point2 &delta) {
    return ts::TileId( tileId.lod
                     , tileId.easting + long(delta(0))
                     , tileId.northing + long(delta(1)));
}

void clone(const ts::Properties &srcProps
          , ts::TileSet &src, vts::TileSet &dst
          , const math::Point2 &delta)
{
    // move alignment to upper-left corner
    auto alignment(srcProps.alignment);
    alignment(1) += srcProps.baseTileSize;

    auto asrc(src.advancedApi());
    auto adst(dst.advancedApi());

    const utility::Progress::ratio_t reportRatio(1, 100);
    utility::Progress progress(asrc.tileCount());

    // process all tiles
    asrc.traverseTiles([&](const ts::TileId &tileId)
    {
        auto dTileId( asVts(alignment, srcProps.baseTileSize
                    , applyDelta(tileId, delta)));
        LOG(info1) << "Converting " << tileId << " -> " << dTileId;
        const auto *metanode(asrc.findMetaNode(tileId));
        if (!metanode) {
            LOG(warn2)
                << "Cannot find metanode for tile " << tileId << "; "
                << "skipping.";
            return;
        }

        // copy mesh and atlas
        for (auto type : { ts::TileFile::mesh, ts::TileFile::atlas }) {
            copyFile(asrc.input(tileId, type)
                     , adst.output(dTileId, asVts(type)));
        }

        adst.setMetaNode(dTileId, asVts(*metanode));
        (++progress).report(reportRatio, "convert ");
    });
}

int TileSet2Vts::run()
{
    LOG(info3)
        << "Converting tileset <" << input_ << "> into <" << output_ << ">.";

    // open source tileset
    auto src(ts::openTileSet(input_));
    const auto props(src->getProperties());

    auto rootExtents
        (math::Extents2
         (props.alignment
          , math::Point2(props.alignment(0) + props.baseTileSize
                         , props.alignment(1) + props.baseTileSize)));

    auto rf(vr::system.referenceFrames(referenceFrame_));

    // check
    if (rootExtents != rf.rootExtents()) {
        if (rootExtents.size() != rf.rootExtents().size()) { 
            LOGTHROW(err3, std::runtime_error)
                << "Reference frame has different extents than extents computed "
                "from alignment and baseTileSize and different size, unable to translate.";
        }
        LOG(warn3) << "Root extents have different origin, translating.";
        rootDelta_ = rootExtents.ll - rf.rootExtents().ll;
        LOG(info2) << "Origin delta is " << rootDelta_;
    }

    // create destination tileset
    vts::CreateProperties cprops;

    auto staticProperties(cprops.staticSetter());
    staticProperties.id(props.id);
    staticProperties.srs(props.srs);
    staticProperties.metaLevels(asVts(props.metaLevels));
    staticProperties.extents(rf.rootExtents());
    staticProperties.referenceFrame(referenceFrame_);

    auto settableProperties(cprops.settableSetter());
    settableProperties.textureQuality(props.textureQuality);
    settableProperties.defaultPosition(props.defaultPosition);
    settableProperties.defaultOrientation(props.defaultOrientation);
    settableProperties.texelSize(props.texelSize);

    auto dst(vts::createTileSet(output_, cprops, createMode_));

    clone(props, *src, *dst, rootDelta_);

    dst->flush();

    LOG(info3)
        << "Tileset <" << input_ << "> converted into <" << output_ << ">.";
    return EXIT_SUCCESS;
}

} // namespace

int main(int argc, char *argv[])
{
    return TileSet2Vts()(argc, argv);
}
