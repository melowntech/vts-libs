#include <cstdlib>
#include <string>
#include <iostream>

#include "dbglog/dbglog.hpp"

#include "utility/gccversion.hpp"

#include "service/cmdline.hpp"

#include "./commands.hpp"

namespace po = boost::program_options;

namespace {

class Fallback : public service::Cmdline
{
public:
    Fallback()
        : service::Cmdline("vadstena-storage", BUILD_TARGET_VERSION
                           , (service::ENABLE_UNRECOGNIZED_OPTIONS
                              | service::DISABLE_EXCESSIVE_LOGGING))
    {}

private:
    virtual void configuration(po::options_description &cmdline
                               , po::options_description &config
                               , po::positional_options_description &pd)
        UTILITY_OVERRIDE;

    virtual void configure(const po::variables_map &vars)
        UTILITY_OVERRIDE;

    virtual void configure(const std::vector<std::string> &)
        UTILITY_OVERRIDE {}

    virtual bool help(std::ostream &out, const std::string &what) const
        UTILITY_OVERRIDE;

    virtual int run() UTILITY_OVERRIDE;
};

void Fallback::configuration(po::options_description &cmdline
                           , po::options_description &config
                           , po::positional_options_description &pd)
{
    (void) cmdline;
    (void) config;
    (void) pd;
}

void Fallback::configure(const po::variables_map &vars)
{
    (void) vars;
}

bool Fallback::help(std::ostream &out, const std::string &what) const
{
    if (what.empty()) {
        out << R"RAW(Vadstena storage manipulation program
usage
    vadstena-storage STORAGE/TILESET COMMAND [OPTION]

where COMMAND is one of:
    create                      create new storage
    info                        show information about existing storage
    add                         add tile set to storage
    tileset-info                show information about existing tile set
    tileset-paste               pastes one or more tiles into existing tileset
    tileset-clone               clones existing tileset

To get command help run:
    vadstena-storage STORAGE/TILESET COMMAND --help
)RAW";
    }
    return false;
}

int Fallback::run()
{
    std::cerr << "Invalid invocation. Use --help to get help."
              << std::endl;
    return EXIT_FAILURE;
}

} // namespace

int fallback(int argc, char *argv[])
{
    return Fallback()(argc, argv);
}
