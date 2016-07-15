/**
 * \file registry/freelayer.hpp
 * \author Vaclav Blazek <vaclav.blazek@citationtech.net>
 */

#ifndef vadstena_libs_registry_freelayer_hpp_included_
#define vadstena_libs_registry_freelayer_hpp_included_

#include <set>
#include <map>
#include <string>
#include <vector>
#include <new>

#include <boost/variant.hpp>

#include "./referenceframe.hpp"

namespace vadstena { namespace registry {


struct FreeLayer {
    static constexpr char typeName[] = "free layer";

    enum class Type { external, geodata, geodataTiles, meshTiles };

    struct Geodata {
        // TODO: fill me in
    };

    struct GeodataTiles {
        // TODO: fill me in
    };

    struct MeshTiles {
        LodRange lodRange;
        TileRange tileRange;
        std::string metaUrl;
        std::string meshUrl;
        std::string textureUrl;
    };

    std::string id;
    Type type;
    Credits credits;
    typedef boost::variant<std::string, Geodata
                           , GeodataTiles, MeshTiles> Definition;
    Definition definition;

    typedef Dictionary<FreeLayer> dict;

    template <typename T> T& createDefinition();
};

UTILITY_GENERATE_ENUM_IO(FreeLayer::Type,
    ((external))
    ((geodata))
    ((geodataTiles))
    ((meshTiles))
)


// inlines
template <typename T>
inline T& FreeLayer::createDefinition()
{
    return boost::get<T>(definition = T());
}

} } // namespace vadstena::registry

#endif // vadstena_libs_registry_freelayer_hpp_included_
