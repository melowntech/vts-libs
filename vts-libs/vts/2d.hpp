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
#ifndef vtslibs_vts_2d_hpp_included_
#define vtslibs_vts_2d_hpp_included_

#include <iostream>

#include <boost/gil/gil_all.hpp>

#include "./types2d.hpp"
#include "./mesh.hpp"
#include "./tileindex.hpp"

namespace vtslibs { namespace vts {

typedef boost::gil::gray8_image_t GrayImage;
typedef boost::gil::rgb8_image_t RgbImage;
typedef boost::gil::rgba8_image_t RgbaImage;

GrayImage mask2d(const Mesh::CoverageMask &coverageMask
                 , const std::vector<SubMesh::SurfaceReference>
                 &surfaceReferences
                 , bool singleSourced = false);

GrayImage mask2d(const MeshMask &mask, bool singleSourced = false);

GrayImage meta2d(const TileIndex &tileIndex, const TileId &tileId);

void saveCreditTile(std::ostream &out, const CreditTile &creditTile
                    , bool inlineCredits = true);

CreditTile loadCreditTile(std::istream &in, const boost::filesystem::path &path
                          = "unknown");

RgbaImage debugMask(const Mesh::CoverageMask &coverageMask
                    , const std::vector<SubMesh::SurfaceReference>
                    &surfaceReferences
                    , bool singleSourced = false);

RgbaImage debugMask(const MeshMask &mask, bool singleSourced = false);

RgbaImage debugMask(const Mesh &mesh, bool singleSourced = false);

RgbaImage emptyDebugMask();

RgbaImage fullDebugMask();

// inlines

inline GrayImage mask2d(const MeshMask &mask, bool singleSourced) {
    return mask2d(mask.coverageMask, mask.surfaceReferences, singleSourced);
}

inline RgbaImage debugMask(const MeshMask &mask, bool singleSourced) {
    return debugMask(mask.coverageMask, mask.surfaceReferences, singleSourced);
}

} } // vtslibs::vts

#endif // vtslibs_vts_2d_hpp_included_
