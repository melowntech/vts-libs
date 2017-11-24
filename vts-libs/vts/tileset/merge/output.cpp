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
/**
 * \file vts/meshopinput.hpp
 * \author Vaclav Blazek <vaclav.blazek@citationtech.net>
 *
 * Mesh operation input.
 */


#include "../../meshop.hpp"
#include "../merge.hpp"

namespace vtslibs { namespace vts { namespace merge {

Mesh& Output::forceMesh()
{
    if (!mesh) {
        // mesh with empty mask
        mesh = std::make_shared<Mesh>(false);
    }
    return *mesh;
}

opencv::HybridAtlas& Output::forceAtlas()
{
    if (!atlas) {
        atlas = std::make_shared<opencv::HybridAtlas>();
    }
    return *atlas;
}

opencv::NavTile& Output::forceNavtile()
{
    if (!navtile) {
        navtile = std::make_shared<opencv::NavTile>();
    }
    return *navtile;
}

Tile Output::tile(int textureQuality)
{
    Tile tile;

    if (textureQuality && mesh) {
        // we have mesh and should generate textures, try to optimize it

        // wrap mesh and atlas in shared pointers
        Mesh::pointer m;
        opencv::HybridAtlas::pointer a;
        if (atlas) { a.reset(&*atlas, [](void*) {}); }
        if (mesh) { m.reset(&*mesh, [](void*) {}); }

        if (textureQuality) {
            // optimize
            auto optimized(mergeSubmeshes(tileId, m, a, textureQuality));
            // assign output
            tile.mesh = std::get<0>(optimized);
            tile.atlas = std::get<1>(optimized);
        } else {
            // no optimization
            tile.mesh = m;
            tile.atlas = a;
        }
    } else {
        // nothing, just wrap members into output
        if (mesh) { tile.mesh.reset(&*mesh, [](void*) {}); }
        if (atlas) { tile.atlas.reset(&*atlas, [](void*) {}); }
    }

    if (navtile) { tile.navtile.reset(&*navtile, [](void*) {}); }

    // join all credits from tile mesh source
    for (const auto &src : source.mesh) {
        const auto &sCredits(src.node().credits());
        tile.credits.insert(sCredits.begin(), sCredits.end());
    }

    // update geom extents
    tile.geomExtents = geomExtents;

    return tile;
}

} } } // namespace vtslibs::vts::merge
