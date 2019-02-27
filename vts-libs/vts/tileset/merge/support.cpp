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

#include "dbglog/dbglog.hpp"

#include "support.hpp"

namespace vtslibs { namespace vts { namespace merge {

Input::list filterSources(const Input::list &reference
                          , const Input::list &sources)
{
    Input::list out;
    auto ireference(reference.begin()), ereference(reference.end());
    auto isources(sources.begin()), esources(sources.end());

    // place inputs from sources to output only when present in reference
    while ((ireference != ereference) && (isources != esources)) {
        const auto &r(*ireference);
        const auto &s(*isources);
        if (r < s) {
            ++ireference;
        } else if (s < r) {
            ++isources;
        } else {
            out.push_back(s);
            ++ireference;
            ++isources;
        }
    }
    return out;
}

Vertices3List inputCoverageVertices(const Input &input
                                    , const NodeInfo &nodeInfo
                                    , const CsConvertor &conv
                                    , int margin)
{
    const auto trafo(input.sd2Coverage(nodeInfo, margin));

    const auto &mesh(input.mesh());
    Vertices3List out(mesh.submeshes.size());
    auto iout(out.begin());
    for (const auto &sm : mesh.submeshes) {
        auto &ov(*iout++);
        for (const auto &v : sm.vertices) {
            ov.push_back(transform(trafo, conv(v)));
        }
    }
    return out;
}

math::Matrix4 geo2mask(const math::Extents2 &extents
                       , const math::Size2 &gridSize
                       , int margin)
{
    const auto es(size(extents));

    // scales
    const math::Size2f scale(gridSize.width / es.width
                             , gridSize.height / es.height);

    math::Matrix4 trafo(boost::numeric::ublas::identity_matrix<double>(4));

    // scale to grid
    trafo(0, 0) = scale.width;
    trafo(1, 1) = -scale.height;

    // move to origin
    trafo(0, 3) = -extents.ll(0) * scale.width + margin;
    trafo(1, 3) = extents.ur(1) * scale.height + margin;

    return trafo;
}

math::Matrix4 mask2geo(const math::Extents2 &extents
                       , const math::Size2 &gridSize
                       , int margin)
{
    const auto es(size(extents));

    // scales
    const math::Size2f scale(es.width / gridSize.width
                             , es.height / gridSize.height);

    math::Matrix4 trafo(boost::numeric::ublas::identity_matrix<double>(4));

    // scale to grid
    trafo(0, 0) = scale.width;
    trafo(1, 1) = -scale.height;

    // move to origin
    trafo(0, 3) = extents.ll(0) - margin * scale.width;
    trafo(1, 3) = extents.ur(1) + margin * scale.height;

    return trafo;
}

math::Matrix3 etcNCTrafo(const TileId &id)
{
    math::Matrix3 trafo(boost::numeric::ublas::identity_matrix<double>(3));

    // LOD=0 -> identity
    if (!id.lod) { return trafo; }

    double tileCount(1 << id.lod);

    // number of tiles -> scale
    trafo(0, 0) = tileCount;
    trafo(1, 1) = tileCount;

    // NB: id.x is unsigned -> must cast to double first
    trafo(0, 2) = - double(id.x);
    trafo(1, 2) = (id.y + 1) - tileCount;

    return trafo;
}

math::Matrix4 coverage2EtcTrafo(const math::Size2 &gridSize, int margin)
{
    math::Matrix4 trafo(boost::numeric::ublas::identity_matrix<double>(4));

    // scale to normalized range (0, 1)
    trafo(0, 0) = 1.0 / gridSize.width;
    trafo(1, 1) = -1.0 / gridSize.height;

    // shift to proper orientation
    trafo(0, 3) = -(double(margin) / gridSize.width);
    trafo(1, 3) = 1.0 + (double(margin) / gridSize.height);

    return trafo;
}

} } } // namespace vtslibs::vts::merge
