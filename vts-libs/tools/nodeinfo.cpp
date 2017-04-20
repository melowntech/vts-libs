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

#include "utility/gccversion.hpp"
#include "utility/buildsys.hpp"
#include "service/cmdline.hpp"

#include "../registry/po.hpp"
#include "../vts/basetypes.hpp"
#include "../vts/io.hpp"
#include "../vts/tileop.hpp"

namespace po = boost::program_options;
namespace fs = boost::filesystem;
namespace vr = vtslibs::registry;
namespace vts = vtslibs::vts;

class NodeInfo : public service::Cmdline
{
public:
    NodeInfo()
        : Cmdline("vts-nodeinfo", BUILD_TARGET_VERSION
                  , service::DISABLE_EXCESSIVE_LOGGING)
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
};

void NodeInfo::configuration(po::options_description &cmdline
                             , po::options_description &config
                             , po::positional_options_description &pd)
{
    vr::registryConfiguration(cmdline, vr::defaultPath());

    cmdline.add_options()
        ("referenceFrame", po::value(&referenceFrame_)->required()
         , "Reference frame to query.")
        ("tileId", po::value(&tileId_)->required()
         , "Tile ID to query.")
    ;

    pd.add("referenceFrame", 1)
        .add("tileId", 1);

    (void) config;
}

void NodeInfo::configure(const po::variables_map &vars)
{
    vr::registryConfigure(vars);
}

bool NodeInfo::help(std::ostream &out, const std::string &what) const
{
    if (what.empty()) {
        out << R"RAW(vts-nodeinfo: referenceFrame tileId
)RAW";
    }
    return false;
}

int NodeInfo::run()
{
    const auto referenceFrame(vr::system.referenceFrames(referenceFrame_));
    vts::NodeInfo ni(referenceFrame, tileId_);

    if (!ni.valid()) {
        std::cout << "Node: " << tileId_ << " is not a valid node in "
            "this division space" << std::endl;
        return EXIT_FAILURE;
    };

    std::cout << "Node: " << tileId_ << std::endl;
    std::cout << "Division SRS: " << ni.srs() << std::endl;
    std::cout << "Division extents: "
              << std::fixed << ni.extents() << std::endl;
    std::cout << "Tile size: "
              << std::fixed << size(ni.extents()) << std::endl;
    std::cout << "Subtree root node: " << ni.subtree().id() << std::endl;
    std::cout << "Subtree root division extents: "
              << ni.subtree().root().extents << std::endl;

    std::cout << "Parent: " << vts::parent(tileId_) << std::endl;
    std::cout << "Children: (structure "
              << std::bitset<4>(ni.node().structure.children)
              << ")" << std::endl;
    for (auto child : vts::children(tileId_)) {
        auto childNode(ni.child(child));
        if (childNode.valid()) {
            std::cout << "    " << ni.child(child).nodeId() << std::endl;
        }
    }

    return EXIT_SUCCESS;
}

int main(int argc, char *argv[])
{
    return NodeInfo()(argc, argv);
}
