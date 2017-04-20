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

#include "../registry/po.hpp"
#include "../vts/basetypes.hpp"
#include "../vts/io.hpp"
#include "../vts/tileop.hpp"

namespace po = boost::program_options;
namespace fs = boost::filesystem;
namespace vr = vtslibs::registry;
namespace vts = vtslibs::vts;

class NodeMask : public service::Cmdline
{
public:
    NodeMask()
        : Cmdline("vts-nodemask", BUILD_TARGET_VERSION
                  , service::DISABLE_EXCESSIVE_LOGGING)
        , size_(256, 256)
        , type_(vts::NodeInfo::CoverageType::pixel)
        , dilation_(0)
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
    vts::TileId tileId_;
    math::Size2 size_;
    vts::NodeInfo::CoverageType type_;
    int dilation_;
    fs::path output_;
};

void NodeMask::configuration(po::options_description &cmdline
                             , po::options_description &config
                             , po::positional_options_description &pd)
{
    vr::registryConfiguration(cmdline, vr::defaultPath());

    cmdline.add_options()
        ("referenceFrame", po::value(&referenceFrame_)->required()
         , "Reference frame to query.")
        ("tileId", po::value(&tileId_)->required()
         , "Tile ID to query.")
        ("size", po::value(&size_)->default_value(size_)->required()
         , "Size of mask.")
        ("type", po::value(&type_)->default_value(type_)->required()
         , "Mask type: pixel or grid registration.")
        ("dilation", po::value(&dilation_)
         ->default_value(dilation_)->required()
         , "Mask dilation in pixels.")
        ("output", po::value(&output_)->required()
         , "Path to output file.")
    ;

    pd.add("referenceFrame", 1)
        .add("tileId", 1)
        .add("output", 1);

    (void) config;
}

void NodeMask::configure(const po::variables_map &vars)
{
    vr::registryConfigure(vars);

    if (output_.filename() == ".") {
        output_ /= str(boost::format("%s.png") % tileId_);
    }
}

bool NodeMask::help(std::ostream &out, const std::string &what) const
{
    if (what.empty()) {
        out << R"RAW(vts-nodemask: referenceFrame tileId output [options]
)RAW";
    }
    return false;
}

int NodeMask::run()
{
    const auto referenceFrame(vr::Registry::referenceFrame(referenceFrame_));
    vts::NodeInfo ni(referenceFrame, tileId_);

    cv::imwrite
        (output_.string()
         , asCvMat(ni.coverageMask(type_, size_, dilation_))
         , { CV_IMWRITE_JPEG_QUALITY, 100, CV_IMWRITE_PNG_COMPRESSION, 9 });

    return EXIT_SUCCESS;
}

int main(int argc, char *argv[])
{
    return NodeMask()(argc, argv);
}
