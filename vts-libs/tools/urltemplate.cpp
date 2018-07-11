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

#include "dbglog/dbglog.hpp"

#include "utility/gccversion.hpp"
#include "utility/buildsys.hpp"

#include "service/cmdline.hpp"

#include "../registry/po.hpp"
#include "../vts/urltemplate.hpp"
#include "../vts/urltemplate-po.hpp"

namespace po = boost::program_options;
namespace fs = boost::filesystem;
namespace vts = vtslibs::vts;
namespace vr = vtslibs::registry;
namespace vs = vtslibs::storage;
namespace ba = boost::algorithm;

class UrlTemplate : public service::Cmdline {
public:
    UrlTemplate()
        : service::Cmdline("urltemplate", BUILD_TARGET_VERSION
                           , (service::DISABLE_EXCESSIVE_LOGGING))
        , vars_(vts::TileId())
    {}

    ~UrlTemplate() {}

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

    std::string template_;
    vts::UrlTemplate::Vars vars_;
};

void UrlTemplate::configuration(po::options_description &cmdline
                               , po::options_description &config
                               , po::positional_options_description &pd)
{
    vr::registryConfiguration(cmdline, vr::defaultPath());
    vts::configuration(vars_, cmdline);

    cmdline.add_options()
        ("template", po::value(&template_)->required()
         , "URL template to expand.")
        ;

    pd.add("template", 1)
        .add("referenceFrame", 1)
        ;

    (void) config;
}

void UrlTemplate::configure(const po::variables_map &vars)
{
    vr::registryConfigure(vars);
    vts::configure(vars_, vars);
}

bool UrlTemplate::help(std::ostream &out, const std::string &what) const
{
    if (what.empty()) {
        // program help
        out << ("URL template expansion test tool\n"
                );

        return true;
    }

    return false;
}

int UrlTemplate::run()
{
    vts::UrlTemplate(template_).expand(std::cout, vars_);
    std::cout << std::endl;
    return EXIT_SUCCESS;
}

int main(int argc, char *argv[])
{
    return UrlTemplate()(argc, argv);
}
