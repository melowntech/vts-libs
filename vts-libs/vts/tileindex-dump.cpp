#include <boost/filesystem.hpp>

#include "dbglog/dbglog.hpp"

#include "../storage/error.hpp"

#include "./tileindex.hpp"

namespace vadstena { namespace vts {

namespace fs = boost::filesystem;

namespace {

double pixelSize(const math::Size2 &dims, const long maxArea)
{
    long a(long(dims.width) * long(dims.height));
    auto scale(std::sqrt(double(maxArea) / double(a)));
    return scale;
}

} // namespace

void dumpAsImages(const fs::path &path, const TileIndex &ti
                  , TileIndex::Flag::value_type type, const long maxArea)
{
    LOG(info2) << "Dumping tileIndex as image stack at " << path << ".";
    create_directories(path);

    if (ti.trees().empty()) { return; }

    auto lod(ti.lodRange().max);
    const auto &trees(ti.trees());

    for (auto itrees(trees.rbegin()), etrees(trees.rend());
         itrees != etrees; ++itrees)
    {
        LOG(info1) << "Dumping lod " << lod;

        // rasterize and dump
        auto file(path / str(boost::format("%02d.png") % lod));
        dump(*itrees, file, [type](TileIndex::Flag::value_type value)
        {
            return value & type;
        }, pixelSize(itrees->size(), maxArea));

        // next level
        --lod;
    }
}

} } // namespace vadstena::vts
