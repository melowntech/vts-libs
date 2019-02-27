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

#include "service/cmdline.hpp"

#include "../../vts0.hpp"
#include "../../vts0/io.hpp"
#include "../../vts0/tileset-advanced.hpp"

#include "commands.hpp"

namespace fs = boost::filesystem;
namespace po = boost::program_options;
namespace vts = vtslibs::vts0;

namespace {

struct TileIdReader {
    boost::optional<vts::TileId> id;

    operator bool() const { return bool(id); }
};

template<typename CharT, typename Traits>
inline std::basic_istream<CharT, Traits>&
operator>>(std::basic_istream<CharT, Traits> &is, TileIdReader &r)
{
    using boost::spirit::qi::auto_;
    using boost::spirit::qi::omit;
    using boost::spirit::qi::match;

    vts::TileId tid;

    is >> match((auto_ >> omit['-'] >> auto_ >> omit['-'] >> auto_)
                , tid.lod, tid.x, tid.y);
    r.id = tid;

    return is;
}

struct TileIdWriter {
    vts::TileId id;
};

template<typename CharT, typename Traits>
inline std::basic_ostream<CharT, Traits>&
operator<<(std::basic_ostream<CharT, Traits> &os, const TileIdWriter &w)
{
    return os << w.id.lod << "-" << w.id.x << "-" << w.id.y;
}

struct ChildrenWriter {
    vts::Children children;
};

template<typename CharT, typename Traits>
inline std::basic_ostream<CharT, Traits>&
operator<<(std::basic_ostream<CharT, Traits> &os, const ChildrenWriter &w)
{
    for (const auto &c : w.children) {
        os << TileIdWriter{c} << " ";
    }
    return os;
}

class TileSetInfo : public service::Cmdline
{
public:
    TileSetInfo(const fs::path &path)
        : service::Cmdline("vtslibs-storage", BUILD_TARGET_VERSION)
        , path_(path), coarseness_(false), gsd_(false)
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

    const fs::path path_;

    TileIdReader tileId_;

    bool coarseness_;
    bool gsd_;
};

void TileSetInfo::configuration(po::options_description &cmdline
                           , po::options_description &config
                           , po::positional_options_description &pd)
{
    cmdline.add_options()
        ("tileId", po::value(&tileId_), "Tile id to inspect.")
        ("coarseness", "Show coarseness histogram.")
        ("gsd", "Show GSD histogram.")
        ;

    (void) config;
    (void) pd;
}

void TileSetInfo::configure(const po::variables_map &vars)
{
    coarseness_ = vars.count("coarseness");
    gsd_ = vars.count("gsd");
}

bool TileSetInfo::help(std::ostream &out, const std::string &what) const
{
    if (what.empty()) {
        out << R"RAW(tileset-info command
usage
    vtslibs-storage TILE_SET tileset-info [OPTION]
)RAW";
    }
    return false;
}

std::map<double, int> coarsenessHistogram(vts::TileSet &tileSet)
{
    std::map<double, int> hs;

    // get advanced API from tileset
    auto aa(tileSet.advancedApi());
    aa.traverseTiles([&](const vts::TileId &tileId)
    {
        ++hs[tileSet.getMetadata(tileId).coarseness];
    });

    return hs;
}

std::map<double, int> gsdHistogram(vts::TileSet &tileSet)
{
    std::map<double, int> hs;

    // get advanced API from tileset
    auto aa(tileSet.advancedApi());
    aa.traverseTiles([&](const vts::TileId &tileId)
    {
        ++hs[tileSet.getMetadata(tileId).gsd];
    });

    return hs;
}

int TileSetInfo::run()
{
    auto storage(vts::openTileSet(path_, vts::OpenMode::readOnly));

    auto properties(storage->getProperties());
    auto stat(storage->stat());

    std::cout << "Storage: " << path_
              << "\nProperties:\n"
              << std::fixed << utility::dump(properties, "    ")
              << "\nLOD range: " << storage->lodRange()
              << "\ntile count: " << stat.tileCount
              << "\nmetatile count: " << stat.metatileCount
              << "\n"
        ;

    if (tileId_) {
        // show information about given tile
        auto tile(storage->getTile(*tileId_.id));

        double meshArea, atlasArea;
        std::tie(meshArea, atlasArea) = area(tile);

        std::cout << "\n\nTile " << *tileId_.id << ":";
        std::cout << "\n\tsize: " << tileSize(properties, tileId_.id->lod);
        std::cout << "\n\tz-axis range: " << tile.metanode.zmin
                  << ", " << tile.metanode.zmax;
        std::cout << std::fixed
                  << "\n\tgsd: "<< tile.metanode.gsd
                  << "\n\tcoarseness: "<< tile.metanode.coarseness
                  << "\n\tpixelSize: "
                  << "\n\t\t" << tile.metanode.pixelSize[0][0]
                  << ", "  << tile.metanode.pixelSize[0][1]
                  << "\n\t\t" << tile.metanode.pixelSize[1][0]
                  << ", "  << tile.metanode.pixelSize[1][1]
                  << "\n\tatlas size: " << tile.atlas.cols
                  << "x" << tile.atlas.rows <<  " px"
                  << "\n\tmesh:"
                  << "\n\t\tvertices: " << tile.mesh.vertices.size()
                  << "\n\t\ttriangles: " << tile.mesh.facets.size()
                  << "\n\tmesh area: " << meshArea
                  << "\n\tatlas area: " << atlasArea
                  << "\n"
            ;

        std::cout << "\nParent: "
                  << TileIdWriter{parent(*tileId_.id)}
                  << "\nChildren: " << ChildrenWriter
                         {children(*tileId_.id)}
                  << "\n"
            ;
    }

    if (coarseness_) {
        std::cout << "\nCoarseness histogram:\n";
        for (const auto &item : coarsenessHistogram(*storage)) {
            std::cout << "    " << item.first << ": " << item.second
                      << " tiles\n";
        }
    }

    if (gsd_) {
        std::cout << "\nGSD histogram:\n";
        for (const auto &item : gsdHistogram(*storage)) {
            std::cout << "    " << item.first << ": " << item.second
                      << " tiles\n";
        }
    }

    return EXIT_SUCCESS;
}

} // namespace

int tileSetInfo(int argc, char *argv[], const fs::path &path)
{
    return TileSetInfo(path)(argc, argv);
}
