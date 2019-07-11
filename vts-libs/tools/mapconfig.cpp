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
#include <unistd.h>

#include <cerrno>

#include <boost/format.hpp>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/predicate.hpp>

#include <opencv2/highgui/highgui.hpp>

#include "dbglog/dbglog.hpp"

#include "utility/enum-io.hpp"
#include "utility/gccversion.hpp"
#include "utility/streams.hpp"
#include "utility/time.hpp"
#include "utility/filedes.hpp"
#include "utility/uri.hpp"
#include "utility/implicit-value.hpp"

#include "service/cmdline.hpp"

#include "math/io.hpp"

#include "../registry/po.hpp"
#include "../registry/urlexpander.hpp"
#include "../vts.hpp"
#include "../vts/io.hpp"
#include "../vts/csconvertor.hpp"

#include "support/urlfetcher.hpp"

namespace po = boost::program_options;
namespace fs = boost::filesystem;
namespace vts = vtslibs::vts;
namespace vr = vtslibs::registry;
namespace vs = vtslibs::storage;
namespace tools = vtslibs::vts::tools;
namespace ba = boost::algorithm;

UTILITY_GENERATE_ENUM(Command,
                      ((info))
                      ((convert))
                      ((save))
                      )

struct Verbosity {
    int level;
    Verbosity(int level = 0) : level(level) {}
    operator int() const { return level; }
};

void validate(boost::any &v, const std::vector<std::string>&, Verbosity*, int)
{
    if (v.empty()) {
        v = Verbosity(1);
    } else {
        ++boost::any_cast<Verbosity&>(v).level;
    }
}

typedef service::UnrecognizedParser UP;

class MapConfig : public service::Cmdline {
public:
    MapConfig()
        : service::Cmdline("mapconfig", BUILD_TARGET_VERSION
                           , (service::DISABLE_EXCESSIVE_LOGGING
                              | service::ENABLE_UNRECOGNIZED_OPTIONS))
        , noexcept_(false), command_(Command::info)
        , verbose_(0), timeout_(5000)
        , expandRemote_(false)
    {
    }

    ~MapConfig() {}

private:
    virtual void configuration(po::options_description &cmdline
                               , po::options_description &config
                               , po::positional_options_description &pd)
        UTILITY_OVERRIDE;

    virtual void configure(const po::variables_map &vars)
        UTILITY_OVERRIDE;

    virtual UP::optional
    configure(const po::variables_map &vars
              , const std::vector<std::string> &unrecognized)
        UTILITY_OVERRIDE;

    virtual po::ext_parser extraParser() UTILITY_OVERRIDE;

    virtual std::vector<std::string> listHelps() const UTILITY_OVERRIDE;

    virtual bool help(std::ostream &out, const std::string &what) const
        UTILITY_OVERRIDE;

    virtual int run() UTILITY_OVERRIDE;

    template<typename Body>
    void createParser(po::options_description &cmdline, Command command
                      , const std::string &help, Body body)
    {
        auto p(std::make_shared<UP>(help));
        body(*p);
        commandParsers_[command] = p;

        auto name(boost::lexical_cast<std::string>(command));
        cmdline.add_options()
            (name.c_str(), ("alias for --command=" + name).c_str())
            ;
    }

    UP::optional getParser(Command command) const;

    void lockConfiguration(po::options_description &config);

    void lockConfigure(const po::variables_map &vars);

    int runCommand();

    int info();

    int convert();

    int save();

    vts::MapConfig loadMapConfig();

    bool noexcept_;
    fs::path path_;
    Command command_;
    Verbosity verbose_;
    long timeout_;

    std::string ssrs_;
    std::string tsrs_;
    fs::path output_;

    boost::optional<std::string> absolutize_;
    boost::optional<std::string> renameView_;
    bool expandRemote_;

    std::map<Command, std::shared_ptr<UP> > commandParsers_;

    boost::optional<tools::UrlFetcher> fetcher_;
};

void MapConfig::configuration(po::options_description &cmdline
                               , po::options_description&
                               , po::positional_options_description &pd)
{
    cmdline.add_options()
        ("path", po::value(&path_)->required()
         , "Path to VTS mapconfig to work with. Can be an HTTP/HTTPS URL.")
        ("command", po::value(&command_)
         ->default_value(Command::info)->required()
         , "Command to run.")
        ("noexcept", "Do not catch exceptions, let the program crash.")
        ("timeout", po::value(&timeout_)->default_value(timeout_)
         , "HTTP timeout, in ms.")
        ;

    pd.add("path", 1);

    createParser(cmdline, Command::info
                 , "--command=info: show mapconfig info"
                 , [&](UP &p)
    {
        p.options.add_options()
            ("verbose,V", po::value<Verbosity>(&verbose_)->zero_tokens()
             , "Verbose output.")
            ;
    });

    createParser(cmdline, Command::convert
                 , "--command=convert: create convertor between two SRS and "
                 "convert points"
                 , [&](UP &p)
    {
        p.options.add_options()
            ("ssrs", po::value(&ssrs_)->required(), "Source SRS.")
            ("tsrs", po::value(&tsrs_)->required(), "Target SRS.")
            ;

        p.positional
            .add("ssrs", 1)
            .add("tsrs", 1);
    });

    createParser(cmdline, Command::save
                 , "--command=save: save map configuration to given file"
                 , [&](UP &p)
    {
        p.options.add_options()
            ("output", po::value(&output_)->required(), "Output file.")
            ("absolutize", utility::implicit_value<std::string>
             (nullptr, std::string())
             , "Absolutize all URLs in the output. Can be a URL of the "
             "source mapconfig. If left empty the original path/URL "
             "from --path is used. Optional.")
            ("renameView", po::value<std::string>()
             , "Default view gets renamed to this name if specified.")
            ("expandRemote"
             , utility::implicit_value<bool>(&expandRemote_, true)
             , "All remote entities (bound layers and free layers) are "
             "downloaded and stored in place.")
            ;

        p.positional
            .add("output", 1);

        p.configure = [&](const po::variables_map &vars) {
            if (vars.count("absolutize")) {
                absolutize_ = vars["absolutize"].as<std::string>();
            }
            if (vars.count("renameView")) {
                renameView_ = vars["renameView"].as<std::string>();
            }
        };
    });
}

po::ext_parser MapConfig::extraParser()
{
    return [&](const std::string &s) -> std::pair<std::string, std::string>
    {
        if ((s.size() < 3) || (s[0] != '-') || (s[1] != '-')) {
            return {};
        }

        // translate standalone --COMMAND_NAME into --command=COMMAND_NAME
        for (const auto &p : commandParsers_) {
            const auto &name(boost::lexical_cast<std::string>(p.first));
            if (!s.compare(2, std::string::npos, name)) {
                return { "command", name };
            }
        }
        return {};
    };
}

UP::optional
MapConfig::configure(const po::variables_map &vars
                 , const std::vector<std::string>&)
{
    if (!vars.count("command")) { return {}; }
    return getParser(vars["command"].as<Command>());
}

UP::optional MapConfig::getParser(Command command)
    const
{
    auto fcommandParsers(commandParsers_.find(command));
    if ((fcommandParsers != commandParsers_.end())
         && fcommandParsers->second)
    {
        return *fcommandParsers->second;
    }

    return {};
}

void MapConfig::configure(const po::variables_map &vars)
{
    noexcept_ = vars.count("noexcept");

    fetcher_ = boost::in_place(timeout_, 1);
}

std::vector<std::string> MapConfig::listHelps() const
{
    std::vector<std::string> out;

    for (const auto &p : commandParsers_) {
        out.push_back(boost::lexical_cast<std::string>(p.first));
    }

    return out;
}

bool MapConfig::help(std::ostream &out, const std::string &what) const
{
    if (what.empty()) {
        // program help
        out << ("Map config manipulator\n"
                );

        return true;
    }

    try {
        if (auto p = getParser(boost::lexical_cast<Command>(what))) {
            out << p->options;
        }
        return true;
    } catch (const boost::bad_lexical_cast&) {}

    return false;
}

int MapConfig::runCommand()
{
    switch (command_) {
    case Command::info: return info();
    case Command::convert: return convert();
    case Command::save: return save();
    }
    std::cerr << "mapconfig: no operation requested" << '\n';
    return EXIT_FAILURE;
}

int MapConfig::run()
{
    if (noexcept_) {
        return runCommand();
    }

    try {
        return runCommand();
    } catch (const std::exception &e) {
        std::cerr << "vts: " << e.what() << '\n';
        return EXIT_FAILURE;
    }
}

void expandRemote(vts::MapConfig &mc, const std::string &baseUrl
                  , const tools::UrlFetcher &fetcher)
{
    // need to copy due to boost multi-index container value constness

    vr::UrlExpander e(baseUrl, [&fetcher](const std::string &url) {
            return fetcher.fetch(url);
        });

    mc.boundLayers = e.expand(mc.boundLayers);
    mc.freeLayers = e.expand(mc.freeLayers);
}

vts::MapConfig MapConfig::loadMapConfig()
{
    vts::MapConfig mc;

    utility::Uri url(path_.string());
    if (url.absolute()) {
        // fetch from remote HTTP
        if (url.scheme().empty()) { url.scheme("http"); }

        std::istringstream is(fetcher_->fetch(url.str()));
        is.exceptions(std::ios::badbit | std::ios::failbit);
        vts::loadMapConfig(mc, is, path_);

    } else {

        std::ifstream f;
        f.exceptions(std::ios::badbit | std::ios::failbit);
        f.open(path_.string(), std::ios_base::in);
        vts::loadMapConfig(mc, f, path_);
    }

    return mc;
}

int MapConfig::info()
{
    auto mc(loadMapConfig());

    std::cout << "referenceFrame: " << mc.referenceFrame.id << "\n";

    std::cout << "srs:";
    for (const auto &item : mc.srs) {
        std::cout << " " << item.first;
    }
    std::cout << "\n";

    std::cout << "nodes:";
    for (const auto &item : mc.referenceFrame.division.nodes) {
        std::cout << " " << item.first << "[" << item.second.srs << "]";
    }
    std::cout << "\n";

    return EXIT_SUCCESS;
}

int MapConfig::convert()
{
    auto mc(loadMapConfig());

    vts::CsConvertor conv(ssrs_, tsrs_, mc);

    return EXIT_SUCCESS;
}

int MapConfig::save()
{
    auto mc(loadMapConfig());

    std::ofstream f;
    f.exceptions(std::ios::badbit | std::ios::failbit);
    f.open(output_.string(), std::ios_base::out);

    auto baseUrl([&]() -> std::string {
            if (absolutize_ && !absolutize_->empty()) {
                return *absolutize_;
            }
            return path_.string();
        }());

    if (absolutize_) {
        vts::absolutize(mc, baseUrl);
    }

    if (expandRemote_) {
        expandRemote(mc, baseUrl, *fetcher_);
    }

    if (renameView_) {
        mc.namedViews[*renameView_] = mc.view;
        mc.view = vr::View();
    }

    vts::saveMapConfig(mc, f);
    f.close();

    return EXIT_SUCCESS;
}

int main(int argc, char *argv[])
{
    return MapConfig()(argc, argv);
}
