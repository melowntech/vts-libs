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
namespace vr = vtslibs::registry;
namespace vts = vtslibs::vts;

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
    auto st(vts::findSubtrees(vr::Registry::referenceFrame(referenceFrame_)
                              , lod_, srs_, extents_, samples_));

    for (const auto &range : st.ranges) {
        std::cout
            << "node " << range.first << ": " << range.second << std::endl;
    }

    if (math::valid(st.overallRange)) {
        std::cout << "overall: " << st.overallRange << std::endl;
    }

    return EXIT_SUCCESS;
}

int main(int argc, char *argv[])
{
    return MeasureDataset()(argc, argv);
}
