#include <cstdlib>
#include <string>
#include <iostream>
#include <algorithm>
#include <iterator>

#include <boost/filesystem.hpp>

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

#include "geo/csconvertor.hpp"
#include "geo/coordinates.hpp"
#include "geo/geodataset.hpp"

#include "../registry/po.hpp"
#include "../vts.hpp"
#include "../vts/encoder.hpp"
#include "../vts/opencv/navtile.hpp"
#include "../vts/io.hpp"
#include "../vts/csconvertor.hpp"
#include "../vts/meshopinput.hpp"
#include "../vts/meshop.hpp"
#include "../vts/heightmap.hpp"
#include "../vts/tileflags.hpp"


namespace po = boost::program_options;
namespace vs = vadstena::storage;
namespace vr = vadstena::registry;
namespace vts = vadstena::vts;
namespace ba = boost::algorithm;
namespace fs = boost::filesystem;
namespace ublas = boost::numeric::ublas;

namespace {

struct Config {

    Config()
        : samplesPerTile(128, 128), geoidGrid("egm96_15.gtx")
    {}

    boost::optional<vts::Lod> lod;
    math::Size2 samplesPerTile;
    boost::optional<std::string> srs;
    std::string geoidGrid;
};

class Vts2Dem : public service::Cmdline
{
public:
    Vts2Dem()
        : service::Cmdline("vts2dem", BUILD_TARGET_VERSION)
        , overwrite_(false)
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

    bool overwrite_;

    Config config_;
};

void Vts2Dem::configuration(po::options_description &cmdline
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

        ("lod", po::value<vts::Lod>()
         , "Rasterize given lod instead of the highest one.")

        ("samplesPerTile", po::value(&config_.samplesPerTile)
         ->default_value(config_.samplesPerTile)->required()
         , "Number of samples per one tile.")

        ("srs", po::value<std::string>()
         , "Selects reference frame subtree to extract. Optional in case of "
         "single-SRS reference frame.")

        ("geoidGrid", po::value(&config_.geoidGrid)
         ->default_value(config_.geoidGrid)->required()
         , "Geoid grid of output dataset.")
        ;

    pd.add("input", 1);
    pd.add("output", 1);

    (void) config;
}

void Vts2Dem::configure(const po::variables_map &vars)
{
    vr::registryConfigure(vars);

    overwrite_ = vars.count("overwrite");

    if (vars.count("lod")) {
        config_.lod = vars["lod"].as<vts::Lod>();
    }
}

bool Vts2Dem::help(std::ostream &out, const std::string &what) const
{
    if (what.empty()) {
        out << R"RAW(vts2dem
usage
    vts2vts INPUT OUTPUT [OPTIONS]

)RAW";
    }
    return false;
}

math::Extents2 geoExtents(const vr::ReferenceFrame &referenceFrame
                         , const vts::Lod lod
                         , const vts::TileRange &tileRange)
{
    vts::NodeInfo ll(referenceFrame, vts::tileId(lod, ul(tileRange)));
    vts::NodeInfo ur(referenceFrame, vts::tileId(lod, lr(tileRange)));

    return math::Extents2(ll.extents().ll, ur.extents().ur);
}

/** Geo coordinates to grid mapping.
 * NB: result is in pixel system (pixel centers have integral indices)
 */
inline math::Matrix4 geo2grid(const math::Extents2 &extents
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

void makeLocal(vts::Mesh &mesh
               , const vts::NodeInfo &nodeInfo
               , const vts::CsConvertor &conv
               , const math::Size2 &gridSize)
{
    const auto trafo(geo2grid(nodeInfo.extents(), gridSize));

    for (auto &sm : mesh.submeshes) {
        for (auto &v : sm.vertices) {
            v = transform(trafo, conv(v));
        }
    }
}

void rasterize(cv::Mat_<double> &pane, const vts::Mesh &mesh)
{
    std::vector<imgproc::Scanline> scanlines;

    for (const auto &sm : mesh.submeshes) {
        for (const auto &face : sm.faces) {

            const math::Point3 *tri[3] = {
                &sm.vertices[face[0]]
                , &sm.vertices[face[1]]
                , &sm.vertices[face[2]]
            };

            imgproc::scanConvertTriangle
                (*tri[0], *tri[1], *tri[2], 0, pane.rows, scanlines);

            for (const auto &sl : scanlines) {
                imgproc::processScanline(sl, 0, pane.cols
                                         , [&](int x, int y, double z)
                {
                    // precondition: no data value is less than anything else
                    auto &old(pane(y, x));
                    old = std::max(old, z);
                });
            }
        }
    }
}

void process(cv::Mat &data, geo::GeoDataset::Mask &mask
             , const vts::TileSet &ts, vts::Lod lod
             , const vts::TileRange &tileRange
             , const Config &config
             , const vts::CsConvertor &phys2sd)
{
    UTILITY_OMP(parallel)
    UTILITY_OMP(single)
    traverse(ts.tileIndex(), lod, [&](vts::TileId tileId
                                      , vts::QTree::value_type flags)
    {
        if (!vts::TileIndex::Flag::isReal(flags)) { return; }

        UTILITY_OMP(task)
        {
            auto mesh([&]() -> vts::Mesh
            {
                vts::Mesh m;
                UTILITY_OMP(critical)
                    m = ts.getMesh(tileId);
                return m;
            }());

            const auto ni(ts.nodeInfo(tileId));
            const auto &size(config.samplesPerTile);

            const vts::TileRange::point_type offset
                ((tileId.x - tileRange.ll(0)) * size.width
                 , (tileId.y - tileRange.ll(1)) * size.height);

            LOG(info4) << tileId << ": " << vts::TileFlags(flags)
                       << " (" << offset << ")";

            LOG(info4) << data.size();

            cv::Mat_<double> pane
                (data, cv::Range(offset(1), offset(1) + size.height)
                 , cv::Range(offset(0), offset(0) + size.width));

            makeLocal(mesh, ni, phys2sd, config.samplesPerTile);

            rasterize(pane, mesh);
        }
    });

    (void) mask;
}

int Vts2Dem::run()
{
    auto input(vts::openTileSet(input_));

    if (!overwrite_ && fs::exists(output_)) {
        LOG(fatal)
            << "Output file " << output_ << " already exists. Use "
            "--overwrite option to overwrite it.";
        return EXIT_FAILURE;
    }

    const auto rf(input.referenceFrame());

    vts::TileId rootId;

    const auto srsList(rf.division.srsList());
    if (!config_.srs) {
        // no SRS configured
        if (srsList.size() != 1) {
            LOG(fatal)
                << "Multi-SRS reference frame but not SRS "
                "specified (use --srs).";
            return EXIT_FAILURE;
        }
        config_.srs = *srsList.begin();
    } else if (srsList.find(*config_.srs) != srsList.end()) {
        LOG(fatal)
            << "SRS " << *config_.srs << " not found in reference frame.";
        return EXIT_FAILURE;
    }

    for (const auto &item : rf.division.nodes) {
        if (item.second.srs == *config_.srs) {
            rootId = item.first;
            break;
        }
    }

    // get node SRS
    const auto srs(geo::setGeoid(vr::system.srs(*config_.srs).srsDef
                                 , config_.geoidGrid));

    const auto lr(input.lodRange());

    if (config_.lod) {
        if (!in(*config_.lod, lr)) {
            LOG(fatal)
                << "LOD " << *config_.lod
                << " is outside of tile set's LOD range " << lr << ".";
            return EXIT_FAILURE;
        }
    } else {
        config_.lod = lr.max;
    }

    const auto tr(input.tileRange(*config_.lod));

    // TODO: clip tile range to tile range under rootId

    if (!valid(tr)) {
        LOG(fatal)
            << "Nothing to rasterize at LOD " << *config_.lod << ".";
        return EXIT_FAILURE;
    }

    const auto sizeInTiles(vts::tileRangesSize(tr));

    const auto extents(geoExtents(rf, *config_.lod, tr));
    const math::Size2 size
        (sizeInTiles.width * config_.samplesPerTile.width
         , sizeInTiles.height * config_.samplesPerTile.height);

    LOG(info3) << std::fixed << "Creating output dataset " << output_
               << " extents: " << extents
               << ", srs: \"" << srs << "\".";
    fs::remove_all(output_);

    const geo::NodataValue ndv(-1e6);

    auto output(geo::GeoDataset::create(output_, srs, extents, size
                                        , geo::GeoDataset::Format::dsm()
                                        , ndv));

    // tile -> SRS covertor
    const vts::CsConvertor phys2sd(rf.model.physicalSrs, srs);

    LOG(info3) << "Rasterizing " << sizeInTiles << "tiles ("
               << tr << ") at LOD " << *config_.lod << ".";

    output.data() = cv::Scalar(*ndv);
    process(output.data(), output.mask(), input, *config_.lod
            , tr, config_, phys2sd);

    output.flush();

    // all done
    LOG(info4) << "All done.";
    return EXIT_SUCCESS;
}

} // namespace

int main(int argc, char *argv[])
{
    return Vts2Dem()(argc, argv);
}

