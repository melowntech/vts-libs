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
#include <queue>
#include <bitset>
#include <iostream>
#include <limits>
#include <algorithm>
#include <cfenv>

#include "dbglog/dbglog.hpp"
#include "utility/binaryio.hpp"
#include "utility/algorithm.hpp"
#include "math/geometry.hpp"
#include "math/math.hpp"
#include "half/half.hpp"

#include "../storage/error.hpp"
#include "tileop.hpp"
#include "io.hpp"
#include "types.hpp"
#include "metatileop.hpp"

namespace vtslibs { namespace vts0 {

namespace detail {

double triangleArea(const math::Point3 &a, const math::Point3 &b,
                    const math::Point3 &c)
{
    return norm_2(math::crossProduct(b - a, c - a)) * 0.5;
}

double pixelSize(const geometry::Obj &mesh, const math::Size2 &atlasSize)
{
    if (mesh.facets.empty()) return detail::invalidPixelSize;

    // calculate the total area of the faces in both the XYZ and UV spaces
    double xyzArea(0), uvArea(0);
    for (const auto &face : mesh.facets)
    {
        xyzArea += triangleArea(mesh.vertices[face.v[0]],
                                mesh.vertices[face.v[1]],
                                mesh.vertices[face.v[2]]);
        uvArea += triangleArea(mesh.texcoords[face.t[0]],
                               mesh.texcoords[face.t[1]],
                               mesh.texcoords[face.t[2]]);
    }
    uvArea *= atlasSize.width * atlasSize.height;

    // no texturing -> invalid tile
    if (!uvArea) {
        return detail::invalidPixelSize;
    }

    return sqrt(xyzArea / uvArea);
}

} // namespace detail

std::pair<double, double> area(const Tile &tile)
{
    if (tile.mesh.facets.empty()) {
        return { 0.0, 0.0 };
    }

    // calculate the total area of the faces in both the XYZ and UV spaces
    double xyzArea(0), uvArea(0);
    for (const auto &face : tile.mesh.facets)
    {
        xyzArea += detail::triangleArea(tile.mesh.vertices[face.v[0]],
                                        tile.mesh.vertices[face.v[1]],
                                        tile.mesh.vertices[face.v[2]]);
        uvArea += detail::triangleArea(tile.mesh.texcoords[face.t[0]],
                                       tile.mesh.texcoords[face.t[1]],
                                       tile.mesh.texcoords[face.t[2]]);
    }
    uvArea *= tile.atlas.cols * tile.atlas.rows;

    return { xyzArea, uvArea };
}

void calcParams(MetaNode &metanode, const geometry::Obj &mesh
                , const math::Size2 &atlasSize
                , const boost::optional<double> &forcePixelSize)
{
    if (mesh.facets.empty()) return;

    // calculate Z range
    metanode.zmin = INFINITY; metanode.zmax = -INFINITY;
    for (const auto &vertex : mesh.vertices)
    {
        if (vertex(2) < metanode.zmin) metanode.zmin = vertex(2);
        if (vertex(2) > metanode.zmax) metanode.zmax = vertex(2);
    }

    // calculate pixel size from atlas or used forced value
    double ps(forcePixelSize
              ? *forcePixelSize
              : detail::pixelSize(mesh, atlasSize));

    // calculate texture resolution
    metanode.pixelSize[0][0] = metanode.pixelSize[0][1] = ps;
    metanode.pixelSize[1][0] = metanode.pixelSize[1][1] = ps;
}

} } // namespace vtslibs::vts0
