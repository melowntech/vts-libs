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
#include "utility/streams.hpp"
#include "utility/gccversion.hpp"

#include "service/runninguntilsignalled.hpp"

#include "service/cmdline.hpp"

#include "../../vts0.hpp"
#include "../../vts0/io.hpp"

#include "commands.hpp"

namespace fs = boost::filesystem;
namespace po = boost::program_options;
namespace vts = vtslibs::vts0;

namespace {

class Add : public service::Cmdline
{
public:
    Add(const fs::path &root)
        : service::Cmdline("vtslibs-storage", BUILD_TARGET_VERSION)
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
    vtslibs-storage STORAGE add TILE-SET* [OPTION]
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
