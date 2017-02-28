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
    std::cout << "Children:" << std::endl;
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
