#include <cstdlib>
#include <string>
#include <iostream>

#include "dbglog/dbglog.hpp"
#include "utility/streams.hpp"

#include "utility/gccversion.hpp"

#include "service/cmdline.hpp"

#include "vts-libs/vts0.hpp"
#include "vts-libs/vts0/io.hpp"

#include "./commands.hpp"

namespace fs = boost::filesystem;
namespace po = boost::program_options;
namespace vts = vtslibs::vts0;

namespace {

class Info : public service::Cmdline
{
public:
    Info(const fs::path &root)
        : service::Cmdline("vtslibs-storage", BUILD_TARGET_VERSION)
        , root_(root)
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

    const fs::path root_;
};

void Info::configuration(po::options_description &cmdline
                           , po::options_description &config
                           , po::positional_options_description &pd)
{
    (void) cmdline;
    (void) config;
    (void) pd;
}

void Info::configure(const po::variables_map &vars)
{
    (void) vars;
}

bool Info::help(std::ostream &out, const std::string &what) const
{
    if (what.empty()) {
        out << R"RAW(add command
usage
    vtslibs-storage STORAGE info [OPTION]
)RAW";
    }
    return false;
}

int Info::run()
{
    auto storage(vts::openStorage(root_, vts::OpenMode::readOnly));
    auto properties(storage->getProperties());

    std::cout
        << "Output:\n    " << properties.outputSet.path << "\n";
    std::cout << "Input:\n";
    for (const auto &desc : properties.inputSets) {
        std::cout << "    " << desc.first << "\n";
    }

    return EXIT_SUCCESS;
}

} // namespace

int info(int argc, char *argv[], const fs::path &root)
{
    return Info(root)(argc, argv);
}
