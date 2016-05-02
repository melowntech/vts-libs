#include <cstdlib>

#include <boost/format.hpp>

#include <opencv2/highgui/highgui.hpp>

#include "utility/gccversion.hpp"
#include "utility/buildsys.hpp"
#include "service/cmdline.hpp"

#include "imgproc/rastermask/cvmat.hpp"

#include "geo/geodataset.hpp"

#include "../registry/po.hpp"
#include "../vts/basetypes.hpp"
#include "../vts/io.hpp"
#include "../vts/tileop.hpp"
#include "../vts/csconvertor.hpp"

namespace po = boost::program_options;
namespace fs = boost::filesystem;
namespace vr = vadstena::registry;
namespace vts = vadstena::vts;

class MeasureDataset : public service::Cmdline
{
public:
    MeasureDataset()
        : Cmdline("vts-measure-dataset", BUILD_TARGET_VERSION
                  , service::DISABLE_EXCESSIVE_LOGGING)
        , lod_(), samples_(20)
    {}

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

    std::string referenceFrame_;
    std::string dataset_;
    math::Extents2 extents_;
    geo::SrsDefinition srs_;
    vts::Lod lod_;
    int samples_;
};

void MeasureDataset::configuration(po::options_description &cmdline
                                   , po::options_description &config
                                   , po::positional_options_description &pd)
{
    vr::registryConfiguration(cmdline, vr::defaultPath());

    cmdline.add_options()
        ("referenceFrame", po::value(&referenceFrame_)->required()
         , "Reference frame to query.")
        ("dataset", po::value(&dataset_)
         , "Path to geo dataset (anything GDAL can handle).")
        ("srs", po::value(&srs_)
         , "Dataset's SRS. Conflicts with dataset.")
        ("extents", po::value(&extents_)
         , "Overrides dataset's extents.")
        ("lod", po::value(&lod_)->required()
         , "Lod to work with.")
        ("samples", po::value(&samples_)->default_value(samples_)->required()
         , "Number of samples per dataset edge.");
    ;

    pd.add("referenceFrame", 1)
        .add("lod", 1);

    (void) config;
}

void MeasureDataset::configure(const po::variables_map &vars)
{
    vr::registryConfigure(vars);

    if (vars.count("dataset")) {
        if (vars.count("srs")) {
            throw po::error("conflicting options dataset+srs");
        }

        auto ds(geo::GeoDataset::open(dataset_));
        srs_ = ds.srs();
        if (!vars.count("extents")) {
            extents_ = ds.extents();
        }
    } else if (!vars.count("srs") && !vars.count("extents")) {
        throw po::required_option("srs+extents");
    }
}

bool MeasureDataset::help(std::ostream &out, const std::string &what) const
{
    if (what.empty()) {
        out << R"RAW(vts-measure-dataset: referenceFrame lod [options]
)RAW";
    }
    return false;
}

int MeasureDataset::run()
{
    const auto referenceFrame(vr::Registry::referenceFrame(referenceFrame_));

    vts::TileRange overall(math::InvalidExtents{});

    // process all nodes
    for (const auto &item : referenceFrame.division.nodes) {
        // skip invalid nodes
        const auto &node(item.second);
        if (!node.real()) { continue; }

        // skip nodes below requested lod
        const auto &nid(node.id);
        if (nid.lod > lod_) { continue; }

        auto nodeRange(vts::childRange
                       ({nid.x, nid.y, nid.x, nid.y}, (lod_ - nid.lod)));

        // check for subtree validity
        vts::NodeInfo ni(referenceFrame, vts::tileId(node.id));
        vts::NodeInfo cni(referenceFrame, vts::tileId(lod_, nodeRange.ll));
        if (ni.subtree() != cni.subtree()) {
            // skipped some node -> not a good node here
            continue;
        }

        math::Extents2 extents(math::InvalidExtents{});

        const vts::CsConvertor conv(srs_, node.srs);
        const auto dss(math::size(extents_));
        math::Size2f px(dss.width / samples_, dss.height / samples_);

        // convert dataset's extents into node's SRS
        for (int j(0); j <= samples_; ++j) {
            auto y(extents_.ll(1) + j * px.height);
            for (int i(0); i <= samples_; ++i) {
                math::Point2 p(extents_.ll(0) + i * px.width, y);
                try {
                    // try to convert point from dataset's SRS into node
                    auto pp(conv(p));

                    // check whether point is inside valid area of node
                    if (ni.inside(pp)) {
                        update(extents, pp);
                    }
                } catch (...) {}
            }
        }

        // skip invalid extents
        if (!valid(extents)) { continue; }

        // convert range to node range

        // tile size in this subtree
        const auto ts(vts::tileSize(node.extents, (lod_ - nid.lod)));

        math::Point2 llDiff(extents.ll(0) - node.extents.ll(0)
                            , node.extents.ur(1) - extents.ur(1));
        math::Point2 urDiff(extents.ur(0) - node.extents.ll(0)
                            , node.extents.ur(1) - extents.ll(1));

        math::Point2 llId(llDiff(0) / ts.width, llDiff(1) / ts.height);
        math::Point2 urId(urDiff(0) / ts.width, urDiff(1) / ts.height);

        auto fix([](double &x, bool up) -> void
        {
            if (math::isInteger(x, 1e-15)) {
                // close enough to be an integer
                x = std::round(x);
            }
            // too far away, floor/ceil
            if (up) {
                if (x < 0) {
                    x = std::floor(x);
                } else {
                    x = std::ceil(x);
                }
            } else {
                if (x < 0) {
                    x = std::ceil(x);
                } else {
                    x = std::floor(x);
                }
            }
        });


        // fix ids
        fix(llId(0), false); fix(llId(1), false);
        fix(urId(0), true); fix(urId(1), true);

        // shift to fit into node
        vts::TileRange r(nodeRange.ll(0) + int(llId(0))
                         , nodeRange.ll(1) + int(llId(1))
                         , nodeRange.ll(0) + int(urId(0))
                         , nodeRange.ll(1) + int(urId(1)));

        // merge into overall tile range
        math::update(overall, r.ll);
        math::update(overall, r.ur);

        std::cout << "node " << nid << ": " << r << std::endl;
    }

    if (math::valid(overall)) {
        std::cout << "overall : " << overall << std::endl;
    }

    return EXIT_SUCCESS;
}

int main(int argc, char *argv[])
{
    return MeasureDataset()(argc, argv);
}
