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
#include <string>
#include <iostream>

#include "dbglog/dbglog.hpp"

#include "utility/gccversion.hpp"

#include "service/cmdline.hpp"

#include "commands.hpp"

namespace po = boost::program_options;

namespace {

class Fallback : public service::Cmdline
{
public:
    Fallback()
        : service::Cmdline("vtslibs-storage", BUILD_TARGET_VERSION
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
    vtslibs-storage STORAGE/TILESET COMMAND [OPTION]

where COMMAND is one of:
    create                      create new storage
    info                        show information about existing storage
    add                         add tile set to storage
    tileset-info                show information about existing tile set
    tileset-paste               pastes one or more tiles into existing tileset
    tileset-clone               clones existing tileset

To get command help run:
    vtslibs-storage STORAGE/TILESET COMMAND --help
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
