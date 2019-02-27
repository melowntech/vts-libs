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
#include <boost/iostreams/filtering_stream.hpp>
#include <boost/iostreams/filter/gzip.hpp>
#include <boost/iostreams/restrict.hpp>

#include "dbglog/dbglog.hpp"

#include "utility/expect.hpp"
#include "utility/binaryio.hpp"

#include "math/math.hpp"
#include "math/geometry.hpp"
#include "math/transform.hpp"

#include "imgproc/scanconversion.hpp"

#include "half/half.hpp"

#include "../storage/error.hpp"

#include "mesh.hpp"
#include "meshio.hpp"
#include "multifile.hpp"
#include "math.hpp"
#include "tileindex.hpp"

namespace fs = boost::filesystem;
namespace bio = boost::iostreams;
namespace bin = utility::binaryio;

namespace half = half_float::detail;

namespace vtslibs { namespace vts {

namespace {

/** Geo coordinates to coverage mask mapping.
 * NB: result is in pixel system: pixel centers have integral indices
 */
math::Matrix4 geo2mask(const math::Extents2 &extents
                              , const math::Size2 &gridSize)
{
    math::Matrix4 trafo(boost::numeric::ublas::identity_matrix<double>(4));

    auto es(size(extents));

    // scales
    math::Size2f scale(gridSize.width / es.width
                       , gridSize.height / es.height);

    // scale to grid
    trafo(0, 0) = scale.width;
    trafo(1, 1) = -scale.height;

    // move to origin
    trafo(0, 3) = -extents.ll(0) * scale.width;
    trafo(1, 3) = extents.ur(1) * scale.height;

    return trafo;
}

void updateCoverage(Mesh::CoverageMask &cm, const SubMesh &sm
                    , const math::Extents2 &sdsExtents
                    , std::uint8_t smIndex)
{
    const auto rasterSize(cm.size());
    auto trafo(geo2mask(sdsExtents, rasterSize));

    std::vector<imgproc::Scanline> scanlines;
    cv::Point3f tri[3];
    for (const auto &face : sm.faces) {
        for (int i : { 0, 1, 2 }) {
            auto p(transform(trafo, sm.vertices[face[i]]));
            tri[i].x = p(0); tri[i].y = p(1); tri[i].z = p(2);
        }

        scanlines.clear();
        imgproc::scanConvertTriangle(tri, 0, rasterSize.height, scanlines);

        for (const auto &sl : scanlines) {
            imgproc::processScanline
                (sl, 0, rasterSize.width, [&](int x, int y, float)
            {
                cm.set(x, y, smIndex + 1);
            });
        }
    }
}

} // namespace

void updateCoverage(Mesh &mesh, const SubMesh &sm
                    , const math::Extents2 &sdsExtents
                    , std::uint8_t smIndex)
{
    updateCoverage(mesh.coverageMask, sm, sdsExtents, smIndex);
}

void generateCoverage(Mesh &mesh, const math::Extents2 &sdsExtents)
{
    mesh.createCoverage(false);

    std::uint8_t smIndex(0);
    for (const auto &sm : mesh) {
        updateCoverage(mesh, sm, sdsExtents, smIndex++);
    }
}

void generateMeshMask(MeshMask &mask, const Mesh &mesh
                      , const math::Extents2 &sdsExtents)
{
    mask.createCoverage(false);
    mask.surfaceReferences.clear();

    std::uint8_t smIndex(0);
    for (const auto &sm : mesh) {
        updateCoverage(mask.coverageMask, sm, sdsExtents, smIndex++);
        mask.surfaceReferences.push_back(sm.surfaceReference);
    }
}

} } // namespace vtslibs::vts
