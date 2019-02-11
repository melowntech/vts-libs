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

#include "geo/srsdef.hpp"

#include "service/cmdline.hpp"

#include "../../vts0.hpp"
#include "../../vts0/io.hpp"

#include "commands.hpp"

namespace fs = boost::filesystem;
namespace po = boost::program_options;
namespace vts = vtslibs::vts0;

namespace {

struct AlignmentReader {
    vts::Alignment alignment;
};

template<typename CharT, typename Traits>
inline std::basic_istream<CharT, Traits>&
operator>>(std::basic_istream<CharT, Traits> &is, AlignmentReader &ar)
{
    using boost::spirit::qi::auto_;
    using boost::spirit::qi::omit;
    using boost::spirit::qi::match;

    is >> match((auto_ >> omit[','] >> auto_)
                , ar.alignment(0)
                , ar.alignment(1));

    return is;
}

class Create : public service::Cmdline
{
public:
    Create(const fs::path &root)
        : service::Cmdline("vtslibs-storage", BUILD_TARGET_VERSION)
        , root_(root), overwrite_(false)
        , properties_()
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
    math::Extents2 rootExtents_;
    geo::SrsDefinition srs_;
    vts::LodLevels metaLevels_;

    bool overwrite_;
    vts::StorageCreateProperties properties_;
};

void Create::configuration(po::options_description &cmdline
                           , po::options_description &config
                           , po::positional_options_description &pd)
{
    auto &settableProperties(properties_.createProperties.settableProperties);
    cmdline.add_options()
        ("metaLevels"
         , po::value(&metaLevels_)->required()
         , "Metatile levels (in format lod/delta).")
        ("rootExtents", po::value(&rootExtents_)->required()
         , "Root extents.")
        ("srs", po::value(&srs_)->required()
         , "Spatial reference system.")

        ("textureQuality", po::value(&settableProperties.props.textureQuality)
         ->default_value(settableProperties.props.textureQuality)
         , "Texture quality.")

        ("overwrite", po::value(&overwrite_)
         ->default_value(false)
         ->implicit_value(true)
         , "Allows overwrite of existing storage.")
        ;

    (void) config;
    (void) pd;
}

void Create::configure(const po::variables_map &vars)
{
    {
        auto ss(properties_.createProperties.staticSetter());
        ss.extents(rootExtents_);
        ss.srs(srs_.as(geo::SrsDefinition::Type::proj4).srs);
        ss.metaLevels(metaLevels_);
    }
    if (vars.count("textureQuality")) {
        properties_.createProperties.settableProperties.mask
            |= vts::SettableProperties::Mask::textureQuality;
    }

}

bool Create::help(std::ostream &out, const std::string &what) const
{
    if (what.empty()) {
        out << R"RAW(create command
usage
    vtslibs-storage STORAGE create [OPTION]
)RAW";
    }
    return false;
}

int Create::run()
{
    vts::CreateMode mode(overwrite_
                        ? vts::CreateMode::overwrite
                        : vts::CreateMode::failIfExists);

    auto storage(vts::createStorage(root_, properties_, mode));

    std::cout << "Successfully create storage at <" << root_
              << ">" << std::endl;

    return EXIT_SUCCESS;
}

} // namespace

int create(int argc, char *argv[], const fs::path &root)
{
    return Create(root)(argc, argv);
}
