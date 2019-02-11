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

#include "../storage/error.hpp"

#include "2d.hpp"
#include "qtree-rasterize.hpp"
#include "tileop.hpp"
#include "gil/colors.hpp"

namespace bgil = boost::gil;

namespace vtslibs { namespace vts {

GrayImage mask2d(const Mesh::CoverageMask &coverageMask
                 , const std::vector<SubMesh::SurfaceReference>
                 &surfaceReferences, bool singleSourced)
{
    auto size(Mask2d::size());
    GrayImage out(size.width, size.height, bgil::gray8_pixel_t(0x00), 0);

    auto outView(view(out));

    auto maskSize(Mask2d::maskSize());
    auto maskView(bgil::subimage_view
                  (outView, 0, 0, maskSize.width, maskSize.height));
    rasterize(coverageMask, maskView);

    // get iterators to flag and surface rows
    auto iflag(outView.row_begin(Mask2d::flagRow));
    auto isurfaceReference(outView.row_begin(Mask2d::surfaceRow));
    // submeshes start at 1
    ++iflag;
    ++isurfaceReference;
    for (const auto &sr : surfaceReferences) {
        *iflag++ = Mask2d::Flag::submesh;
        // force single (first) source if asked to; otherwise use mapping
        *isurfaceReference++ = (singleSourced ? 1 : sr);
    }

    return out;
}

void saveCreditTile(std::ostream &out, const CreditTile &creditTile
                    , bool inlineCredits)
{
    registry::saveCredits(out, creditTile.credits, inlineCredits);
}

CreditTile loadCreditTile(std::istream &in
                          , const boost::filesystem::path &path)
{
    CreditTile creditTile;
    registry::loadCredits(in, creditTile.credits, path);
    return creditTile;
}

void CreditTile::update(const CreditTile &other)
{
    for (const auto &credit : other.credits) {
        credits.set(credit.first, credit.second);
    }
}

void CreditTile::expand(const registry::Credit::dict &dict)
{
    registry::Credits tmpCredits;

    for (const auto &item : credits) {
        if (const auto *credit = dict.get(item.first, std::nothrow)) {
            tmpCredits.set(item.first, *credit);
        }
    }

    std::swap(credits, tmpCredits);
}

namespace {

bgil::rgba8_pixel_t rgb2rgba(const bgil::rgb8_pixel_t &p
                             , const uint8_t opacity = 0xff)
{
    return bgil::rgba8_pixel_t(p[0], p[1], p[2], opacity);
}

} // namespace

RgbaImage debugMask(const Mesh::CoverageMask &coverageMask
                    , const std::vector<SubMesh::SurfaceReference>
                    &surfaceReferences, bool singleSourced)
{
    auto size(Mesh::coverageSize());
    RgbaImage out(size.width, size.height
                  , bgil::rgba8_pixel_t(0x00, 0x00, 0x00, 0x00)
                 , 0);

    auto outView(view(out));

    rasterize(coverageMask, outView
              , [&](QTree::value_type value) -> bgil::rgba8_pixel_t
    {
        // apply mapping
        if (singleSourced) {
            // single sourced -> always 1
            value = 1;
        } else if (value <= surfaceReferences.size()) {
            // value surface reference -> translate
            value = surfaceReferences[value - 1];
        }

        return rgb2rgba(gil::palette256[value]);
    });

    return out;
}

RgbaImage emptyDebugMask()
{
    auto size(Mesh::coverageSize());
    return RgbaImage(size.width, size.height
                     , bgil::rgba8_pixel_t(0xFF, 0xFF, 0xFF, 0x00), 0);
}

RgbaImage fullDebugMask()
{
    auto size(Mesh::coverageSize());
    return RgbaImage(size.width, size.height
                     , bgil::rgba8_pixel_t(0xFF, 0xFF, 0xFF, 0xFF), 0);
}

} } // vtslibs::vts
