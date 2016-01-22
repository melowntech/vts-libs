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

#include "service/cmdline.hpp"

#include "math/transform.hpp"
#include "math/filters.hpp"

#include "imgproc/scanconversion.hpp"
#include "imgproc/jpeg.hpp"

#include "geo/csconvertor.hpp"
#include "geo/coordinates.hpp"

#include "vts-libs/registry/po.hpp"
#include "vts-libs/vts.hpp"
#include "vts-libs/vts/encoder.hpp"
#include "vts-libs/vts/opencv/navtile.hpp"
#include "vts-libs/vts/io.hpp"
#include "vts-libs/vts/csconvertor.hpp"

namespace po = boost::program_options;
namespace vs = vadstena::storage;
namespace vr = vadstena::registry;
namespace vts = vadstena::vts;
namespace ba = boost::algorithm;
namespace fs = boost::filesystem;
namespace ublas = boost::numeric::ublas;

namespace {

struct Config {
    std::string referenceFrame;
};

class Vts2Vts : public service::Cmdline
{
public:
    Vts2Vts()
        : service::Cmdline("vts0vts", BUILD_TARGET_VERSION)
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
         , "Path to input (vts0) tile set.")
        ("output", po::value(&output_)->required()
         , "Path to output (vts) tile set.")
        ("overwrite", "Existing tile set gets overwritten if set.")

        ("referenceFrame", po::value(&config_.referenceFrame)->required()
         , "Destination reference frame. Must be different from input "
         "tileset's referenceFrame.")
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

#if 0
vts::TileIndex computeDstTree(const vts::TileIndex &srcTree
                              , const vr::ReferenceFrame &srcRf
                              , const vr::ReferenceFrame &dstRf)
{
    (void) dstRf;

    vts::TileIndex out;

    traverse(srcTree, [&](const vts::TileId &tid, vts::QTree::value_type flags)
    {
        if (!vts::TileIndex::Flag::isReal(flags)) { return; }

        vts::NodeInfo ni(srcRf, tid);

        LOG(info4) << std::fixed << "tile: " << tid << ", " << ni.node.extents;

        // for each destination node
        for (const auto &item : dstRf.division.nodes) {
            const auto &node(item.second);
            LOG(info4) << "    trying to convert between "
                       << ni.node.srs << " and " << node.srs;
            vts::CsConvertor csconv(ni.node.srs, node.srs);

            auto dstExtents(csconv(ni.node.extents));
            if (!overlaps(dstExtents, node.extents)) { continue; }
            LOG(info4) << std::fixed << "    maps to extents: "
                       << dstExtents << " in srs " << node.srs;
        };
    });

    return out;
}
#endif

std::map<std::string, math::Extents2>
measure(const vts::TileSet &tileset, const vr::ReferenceFrame &dstRf)
{
    // initialize extents mapping
    std::map<std::string, math::Extents2> mapping;
    for (const auto &item : dstRf.division.nodes) {
        mapping[item.second.srs] = math::Extents2(math::InvalidExtents{});
    }

    const auto &srcRf(tileset.referenceFrame());
    traverse(tileset.tileIndex()
             , [&](const vts::TileId &tid, vts::QTree::value_type flags)
    {
        if (!vts::TileIndex::Flag::isReal(flags)) { return; }

        vts::NodeInfo ni(srcRf, tid);

        LOG(info4) << std::fixed << "tile: " << tid << ", " << ni.node.extents;

        // for each destination node
        for (const auto &item : dstRf.division.nodes) {
            const auto &node(item.second);
            LOG(info4) << "    trying to convert between "
                       << ni.node.srs << " and " << node.srs;
            vts::CsConvertor csconv(ni.node.srs, node.srs);

            auto dstExtents(csconv(ni.node.extents));
            if (!overlaps(dstExtents, node.extents)) { continue; }
            dstExtents = intersect(dstExtents, node.extents);

            LOG(info4) << std::fixed << "    maps to extents: "
                       << dstExtents << " in srs " << node.srs;

            // update extents
            update(mapping[node.srs], dstExtents.ll);
            update(mapping[node.srs], dstExtents.ur);
        };
    });

    for (const auto item : mapping) {
        LOG(info4) << std::fixed << item.first << ": " << item.second;
    }

    return mapping;
}

class Encoder : public vts::Encoder {
public:
    Encoder(const boost::filesystem::path &path
            , const vts::TileSetProperties &properties
            , vts::CreateMode mode
            , const vts::TileSet &input
            , const Config &config)
        : vts::Encoder(path, properties, mode)
        , config_(config), input_(input)
    {
        auto extents(measure(input, referenceFrame()));

        setConstraints(Constraints().setExtentsGenerator
                       ([&](const std::string &srs) -> math::Extents2
        {
            LOG(info4) << "";
            return extents.at(srs);
        }).setLodRange(vts::LodRange(0, 20)));
    }

private:
    virtual TileResult
    generate(const vts::TileId &tileId, const vts::NodeInfo &nodeInfo
             , const TileResult&)
        UTILITY_OVERRIDE;

    virtual void finish(vts::TileSet&) UTILITY_OVERRIDE {}

    const Config config_;

    const vts::TileSet &input_;
};

Encoder::TileResult
Encoder::generate(const vts::TileId &tileId, const vts::NodeInfo &nodeInfo
                  , const TileResult&)
{
    (void) tileId;
    (void) nodeInfo;
    return TileResult::Result::noDataYet;
}

int Vts2Vts::run()
{
    auto input(vts::openTileSet(input_));

    auto properties(input.getProperties());
    properties.referenceFrame = config_.referenceFrame;

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
