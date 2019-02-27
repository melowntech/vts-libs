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

#include <boost/optional.hpp>
#include <boost/spirit/include/qi.hpp>
#include <boost/spirit/include/qi_match.hpp>
#include <boost/spirit/include/qi_match_auto.hpp>

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

class TileSetPaste : public service::Cmdline
{
public:
    TileSetPaste(const fs::path &path)
        : service::Cmdline("vtslibs-storage", BUILD_TARGET_VERSION)
        , output_(path)
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

    const boost::filesystem::path output_;

    std::vector<boost::filesystem::path> paths_;
};

void TileSetPaste::configuration(po::options_description &cmdline
                           , po::options_description &config
                           , po::positional_options_description &pd)
{
    cmdline.add_options()
        ("tileSet", po::value(&paths_)->required()
         , "boost::filesystem::path of tile set to paste in.");
        ;

    pd.add("tileSet", -1);

    (void) config;
}

void TileSetPaste::configure(const po::variables_map &vars)
{
    (void) vars;
}

bool TileSetPaste::help(std::ostream &out, const std::string &what) const
{
    if (what.empty()) {
        out << R"RAW(tileset-paste command
usage
    vtslibs-storage TILE_SET paste TILE_SET [TILE_SET ...]

This command pastes tiles from one or more tile sets into existing tile
set. Tile data are copied into result and metadata are generated accordingly.

This operation is simplified merge of tile sets. Be aware that "last tile wins"
strategy is used: if there are more than one tiles with the same tileId the last
one (from last tile set specified on the command line) is placed into the
result.

To be used only to glue together non-overlaping data sets like:
  * webexports from one scene's targets
  * piecewise generated heightfield

To balance between performance and security we copy data into output directly
and only metadata are written inside transaction.
)RAW";
    }
    return false;
}

int TileSetPaste::run()
{
    auto output(vts::openTileSet(output_, vts::OpenMode::readWrite));

    vts::TileSet::list tileSets;
    for (const auto &path : paths_) {
        tileSets.push_back(vts::openTileSet(path));
    }

    service::RunningUntilSignalled running;
    vts::pasteTileSets(output, tileSets, &running);

    return EXIT_SUCCESS;
}

} // namespace

int tileSetPaste(int argc, char *argv[], const fs::path &path)
{
    return TileSetPaste(path)(argc, argv);
}
