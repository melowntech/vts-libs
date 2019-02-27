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
#include <boost/format.hpp>
#include <boost/filesystem.hpp>

#include "dbglog/dbglog.hpp"

#include "../storage/error.hpp"

#include "tileindex.hpp"

namespace vtslibs { namespace vts {

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

} } // namespace vtslibs::vts
