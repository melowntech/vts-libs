/**
 * \file registry/types.hpp
 * \author Vaclav Blazek <vaclav.blazek@citationtech.net>
 */

#ifndef vadstena_libs_registry_types_hpp_included_
#define vadstena_libs_registry_types_hpp_included_

#include <vector>
#include <string>

#include <boost/optional.hpp>

#include "math/geometry_core.hpp"

#include "../storage/range.hpp"

namespace vadstena { namespace registry {

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

        BoundLayerParams(const std::string &id = std::string())
            : id(id), alpha()
        {}

        /** Tells whether these bound layer parameters are complex.
         */
        bool isComplex() const { return bool(alpha); }

        typedef std::vector<BoundLayerParams> list;
    };

    struct FreeLayerParams {
        boost::optional<std::string> style;
        BoundLayerParams::list boundLayers;
    };

    typedef std::map<std::string, BoundLayerParams::list> Surfaces;
    typedef std::map<std::string, FreeLayerParams> FreeLayers;

    boost::optional<std::string> description;
    Surfaces surfaces;
    FreeLayers freeLayers;

    operator bool() const { return !(surfaces.empty() && freeLayers.empty()); }

    BoundLayerParams::list& addSurface(const std::string &id) {
        return surfaces[id];
    }

    void merge(const View &view) {
        surfaces.insert(view.surfaces.begin(), view.surfaces.end());
        freeLayers.insert(view.freeLayers.begin(), view.freeLayers.end());
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

} } // namespace vadstena::registry

#endif // vadstena_libs_registry_types_hpp_included_
