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

typedef math::Extents2_<unsigned int> TileRange;

struct View {
    struct Surface {
        typedef std::vector<Surface> list;

        std::string id;
        registry::StringIdSet boundLayers;

        Surface(const std::string &id) : id(id) {}
    };

    Surface::list surfaces;
    registry::StringIdSet freeLayers;
};

struct NamedView {
    struct BoundLayerParams {
        std::string id;
        boost::optional<double> alpha;

        BoundLayerParams(const std::string &id = std::string())
            : id(id), alpha()
        {}

        /** Tells whether these bound layer parameters are complex.
         */
        bool isComplex() const { return alpha; }

        typedef std::vector<BoundLayerParams> list;
    };
    typedef std::map<std::string, BoundLayerParams::list> Surfaces;


    std::string description;
    Surfaces surfaces;

    typedef std::map<std::string, NamedView> map;
};

struct Roi {
    std::string exploreUrl;
    std::string roiUrl;
    std::string thumbnailUrl;

    typedef std::vector<Roi> list;
};

} } // namespace vadstena::registry

#endif // vadstena_libs_registry_types_hpp_included_
