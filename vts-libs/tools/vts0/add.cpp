#include <cstdlib>
#include <string>
#include <iostream>

#include "dbglog/dbglog.hpp"
#include "utility/streams.hpp"
#include "utility/gccversion.hpp"

#include "service/runninguntilsignalled.hpp"

#include "service/cmdline.hpp"

#include "../../vts0.hpp"
#include "../../vts0/io.hpp"

#include "./commands.hpp"

namespace fs = boost::filesystem;
namespace po = boost::program_options;
namespace vts = vadstena::vts0;

namespace {

class Add : public service::Cmdline
{
public:
    Add(const fs::path &root)
        : service::Cmdline("vadstena-storage", BUILD_TARGET_VERSION)
        , root_(root)
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

    const fs::path root_;

    std::vector<boost::filesystem::path> paths_;
};

void Add::configuration(po::options_description &cmdline
                           , po::options_description &config
                           , po::positional_options_description &pd)
{
    cmdline.add_options()
        ("tileSet", po::value(&paths_)->required()
         , "Path of tile set to add.")
        ;

    pd.add("tileSet", -1);

    (void) config;
}

void Add::configure(const po::variables_map &vars)
{
    (void) vars;
}

bool Add::help(std::ostream &out, const std::string &what) const
{
    if (what.empty()) {
        out << R"RAW(add command
usage
    vadstena-storage STORAGE add TILE-SET* [OPTION]
)RAW";
    }
    return false;
}

extern "C" {

} // extern "C"

int Add::run()
{
    auto storage(vts::openStorage(root_, vts::OpenMode::readOnly));

    service::RunningUntilSignalled running;
    storage->addTileSets(paths_, &running);

    return EXIT_SUCCESS;
}

} // namespace

int add(int argc, char *argv[], const fs::path &root)
{
    return Add(root)(argc, argv);
}
