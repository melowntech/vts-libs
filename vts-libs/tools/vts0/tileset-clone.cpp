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

#include "geo/geodataset.hpp"

#include "service/cmdline.hpp"

#include "../../vts0.hpp"
#include "../../vts0/io.hpp"

#include "commands.hpp"

namespace po = boost::program_options;
namespace vts = vtslibs::vts0;
namespace fs = boost::filesystem;

namespace {

struct FilterSettings {
    boost::optional<math::Extents2> extents;
    boost::optional<vts::LodRange> lodRange;
    boost::optional<fs::path> mask;

    operator bool() const { return extents || lodRange || mask; }
};

class TileSetClone : public service::Cmdline
{
public:
    TileSetClone(const fs::path &path)
        : service::Cmdline("vtslibs-storage", BUILD_TARGET_VERSION)
        , input_(path)
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

    const fs::path input_;

    fs::path output_;

    std::string id_;

    vts::CloneOptions cloneOptions_;
};

void TileSetClone::configuration(po::options_description &cmdline
                           , po::options_description &config
                           , po::positional_options_description &pd)
{
    cmdline.add_options()
        ("output", po::value(&output_)->required()
         , "Path of output tileset.")
        ("overwrite", "Overwrite existing dataset.")
        ("rename", po::value(&id_)
         , "Renames tileset to given name if set.")
        ("extents", po::value<math::Extents2>()
         , "Limit output only to tiles that intersect with given extents")
        ("lodRange", po::value<vts::LodRange>()
         , "Limit output only to tiles in given LOD range.")
        ("mask", po::value<fs::path>()
         , "Limit output only to tiles with centers within valid pixels. "
           "Mask must have pixel size equal to tile size of the coarsest "
           "output LOD and must be aligned onto the same grid.")
        ("id", po::value(&id_)
         , "Id of cloned tileset; original name used if not touched.")
    ;

    pd.add("output", 1);

    (void) config;
}

namespace {

void checkMaskValidity( geo::GeoDataset &mask
                      , const vts::TileSet &src
                      , long minLodTileSize)
{
    auto props(src.getProperties());
    
    if (!geo::areSame(mask.srs(), props.srs)) {
        LOGTHROW(err2, std::runtime_error) 
                << "Mask must have the same SRS as the cloned tileset.";
    }

    const auto &e(mask.extents());
    if (e != snapToGrid(e, props.extents.ll, size(props.extents).width)) {
        LOGTHROW(err2, std::runtime_error) 
                << "Mask extents must be aligned to the lowest LOD of"
                   " the output tileset.";
    }

    auto warped(geo::GeoDataset::deriveInMemory(
            mask, mask.srs(), math::Point2(minLodTileSize, minLodTileSize), e));
    
    mask.warpInto(warped, geo::GeoDataset::Resampling::nearest);
    
    auto warpedBack(geo::GeoDataset::deriveInMemory(
            mask, mask.srs(), boost::none, e));
    
    warped.warpInto(warpedBack, geo::GeoDataset::Resampling::nearest);
    
    const cv::Mat &a(mask.data()), &b(warpedBack.data());
    auto it1(a.begin<char>()), it2(b.begin<char>());
    for (; it1 != a.end<char>(); ++it1, ++it2) {
        if (*it1 != *it2) {
            LOGTHROW(err2, std::runtime_error) 
                    << "Mask pixel size must be the same as the tilesize of "
                       "the lowest LOD of the output tileset.";
        }
    }
}

/** Add new filtering restrictions here
 *  NB: make functions as small as possible.
 */
vts::CloneOptions::Filter
createFilter(const FilterSettings &filterSettings, const vts::TileSet &src)
{
    auto pass([](vts::Lod, const math::Extents2 &) -> bool { return true; });

    vts::CloneOptions::Filter checkLodRange(pass)
                           , checkExtents(pass)
                           , checkMask(pass);

    if (filterSettings.lodRange) {
        // just LOD range
        auto lr(*filterSettings.lodRange);
        checkLodRange = [lr](vts::Lod lod, const math::Extents2&) -> bool
        {
            return in(lod, lr);
        };
    }

    if (filterSettings.extents) {
        // just extents
        auto e(*filterSettings.extents);
        checkExtents = [e](vts::Lod, const math::Extents2 &tileExtents) -> bool
        {
            return overlaps(e, tileExtents);
        };
    }

    if (filterSettings.mask) {
        const auto & srcProps(src.getProperties());

        // determine desired extents
        auto extents(srcProps.extents);

        if (filterSettings.extents && filterSettings.lodRange) {
            extents = intersect(extents, *filterSettings.extents);
        }

        extents = snapToGrid(extents, srcProps.extents.ll
                             , size(srcProps.extents).width);

        auto maskDataset(geo::GeoDataset::createFromFS(*filterSettings.mask));

        LOG(info2) << "Checking mask validity.";
        checkMaskValidity(maskDataset, src, size(srcProps.extents).width);

        auto warpedMask(geo::GeoDataset::deriveInMemory(
            maskDataset, maskDataset.srs(), boost::none
          , extents));

        maskDataset.warpInto(warpedMask, geo::GeoDataset::Resampling::nearest);

        imgproc::quadtree::RasterMask mask(warpedMask.mask());
        geo::GeoTransform geoTrans(warpedMask.geoTransform());

#if 0
        //debug
        {
            auto dbgMask(geo::GeoDataset::create(
                    (*filterSettings.mask).string() + "-test.tif"
                  , warpedMask.srs(), warpedMask.extents(), warpedMask.size()
                  , geo::GeoDataset::Format::coverage(),0));

            dbgMask.data() = warpedMask.data();
            dbgMask.applyMask(warpedMask);
            dbgMask.flush();
            LOG(info2) << "Dumped mask at to " 
                       << (*filterSettings.mask).string() + "-test.tif";
        }
#endif
        
        checkMask = [mask, geoTrans](vts::Lod, const math::Extents2 &tileExtents) -> bool
        {
            double row, col;
            auto c(center(tileExtents));
            geoTrans.geo2rowcol(math::Point3(c(0), c(1)), row, col);
            
            return mask.get(round(col), round(row));
        };
        
    }
    
    return [=](vts::Lod lod, const math::Extents2 &tileExtents) -> bool
    {
        return checkLodRange(lod, tileExtents) 
            && checkExtents(lod, tileExtents) 
            && checkMask(lod, tileExtents);
    };
}

template <typename T>
boost::optional<T> getOptional(const po::variables_map &vars, const char *name)
{
    if (!vars.count(name)) { return boost::none; }
    return vars[name].as<T>();
}

} // namespace

void TileSetClone::configure(const po::variables_map &vars)
{
    cloneOptions_.createMode
        = (vars.count("overwrite")
           ? vts::CreateMode::overwrite
           : vts::CreateMode::failIfExists);

    // get filter settings
    FilterSettings filterSettings;
    filterSettings.extents = getOptional<math::Extents2>(vars, "extents");
    filterSettings.lodRange = getOptional<vts::LodRange>(vars, "lodRange");
    filterSettings.mask = getOptional<fs::path>(vars, "mask");

    // if (filterSettings) {
        cloneOptions_.filterFactory = [filterSettings](vts::TileSet &src)
        {
            return createFilter(filterSettings, src);
        };
    // }

    // set id
    if (!id_.empty()) {
        cloneOptions_.staticSetter().id(id_);
    }
}

bool TileSetClone::help(std::ostream &out, const std::string &what) const
{
    if (what.empty()) {
        out << R"RAW(tileset-clone command
usage
    vtslibs-storage INPUT tileset-clone OUTPUT [OPTIONS]

This command clones existing tileset to new location. Can be used to change
underlying representation (and possibly set different id).
)RAW";
    }
    return false;
}

int TileSetClone::run()
{
    LOG(info3)
        << "Cloning tileset " << input_ << " into <" << output_ << ">.";
    vts::cloneTileSet(output_, input_, cloneOptions_);
    LOG(info3)
        << "Tileset " << input_ << " cloned into <" << output_ << ">.";
    return EXIT_SUCCESS;
}

} // namespace

int tileSetClone(int argc, char *argv[], const fs::path &path)
{
    return TileSetClone(path)(argc, argv);
}
