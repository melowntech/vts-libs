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

#include "service/cmdline.hpp"

#include "vts-libs/vts0.hpp"
#include "vts-libs/vts0/io.hpp"

#include "commands.hpp"

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
