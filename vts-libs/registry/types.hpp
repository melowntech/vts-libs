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
 * \file registry/types.hpp
 * \author Vaclav Blazek <vaclav.blazek@citationtech.net>
 */

#ifndef vtslibs_registry_types_hpp_included_
#define vtslibs_registry_types_hpp_included_

#include <vector>
#include <string>

#include <boost/optional.hpp>
#include <boost/any.hpp>

#include "math/geometry_core.hpp"

#include "../storage/range.hpp"

namespace vtslibs { namespace registry {

using storage::Lod;
using storage::LodRange;
using storage::CreditId;
using storage::CreditIds;
typedef storage::Range<double> HeightRange;

typedef std::set<std::string> StringIdSet;
typedef std::set<int> IdSet;
typedef std::vector<std::string> StringIdList;

typedef math::Extents2_<unsigned int> TileRange;

struct View {
    struct BoundLayerParams {
        std::string id;
        boost::optional<double> alpha;
        boost::any options;

        BoundLayerParams(const std::string &id = std::string())
            : id(id), alpha()
        {}

        /** Tells whether these bound layer parameters are complex.
         */
        bool isComplex() const {
            return (bool(alpha) || !options.empty());
        }

        typedef std::vector<BoundLayerParams> list;
    };

    struct FreeLayerParams {
        boost::optional<std::string> style;
        BoundLayerParams::list boundLayers;
        boost::optional<std::array<double, 3>> depthOffset;
        boost::any options;
    };

    typedef std::map<std::string, BoundLayerParams::list> Surfaces;
    typedef std::map<std::string, FreeLayerParams> FreeLayers;
    typedef std::set<std::string> Bodies;

    boost::optional<std::string> description;
    Surfaces surfaces;
    FreeLayers freeLayers;
    Bodies bodies;
    boost::any options;

    operator bool() const { return !(surfaces.empty() && freeLayers.empty()); }

    BoundLayerParams::list& addSurface(const std::string &id) {
        return surfaces[id];
    }

    FreeLayerParams& addFreeLayer(const std::string &id) {
        return freeLayers[id];
    }

    void merge(const View &view) {
        surfaces.insert(view.surfaces.begin(), view.surfaces.end());
        freeLayers.insert(view.freeLayers.begin(), view.freeLayers.end());
        bodies.insert(view.bodies.begin(), view.bodies.end());
    }

    bool hasSurface(const std::string &surfaceId) const {
        return surfaces.count(surfaceId);
    }

    bool hasSurfaces(const std::vector<std::string> &surfaceIds) const {
        for (const auto &id : surfaceIds) {
            if (!surfaces.count(id)) { return false; }
        }
        return true;
    }

    void addBodies(const Bodies &other) {
        bodies.insert(other.begin(), other.end());
    }

    /** So-called named views
     */
    typedef std::map<std::string, View> map;
};

struct Roi {
    std::string exploreUrl;
    std::string roiUrl;
    std::string thumbnailUrl;

    typedef std::vector<Roi> list;
};

} } // namespace vtslibs::registry

#endif // vtslibs_registry_types_hpp_included_
