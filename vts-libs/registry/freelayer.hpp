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
        math::Extents2 extents;
        int displaySize;
        std::string geodata;
        std::string style;
    };

    struct GeodataTiles {
        LodRange lodRange;
        TileRange tileRange;
        std::string metaUrl;
        std::string geodataUrl;
        std::string style;
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

    typedef StringDictionary<FreeLayer> dict;

    template <typename T> T& createDefinition();

    FreeLayer() = default;
    FreeLayer(std::string id, std::string externalUrl);
};

void saveFreeLayer(std::ostream &out, const FreeLayer &freeLayer);

// inlines

inline FreeLayer::FreeLayer(std::string id, std::string externalUrl)
    : id(std::move(id)), type(Type::external)
    , definition(std::move(externalUrl))
{}

template <>
inline FreeLayer::Geodata& FreeLayer::createDefinition<FreeLayer::Geodata>()
{
    type = Type::geodata;
    return boost::get<Geodata>(definition = Geodata());
}

template <>
inline FreeLayer::GeodataTiles&
FreeLayer::createDefinition<FreeLayer::GeodataTiles>()
{
    type = Type::geodataTiles;
    return boost::get<GeodataTiles>(definition = GeodataTiles());
}

template <>
inline FreeLayer::MeshTiles&
FreeLayer::createDefinition<FreeLayer::MeshTiles>()
{
    type = Type::meshTiles;
    return boost::get<MeshTiles>(definition = MeshTiles());
}

UTILITY_GENERATE_ENUM_IO(FreeLayer::Type,
    ((external))
    ((geodata))
    ((geodataTiles)("geodata-tiles"))
    ((meshTiles)("mesh-tiles")))

} } // namespace vadstena::registry

#endif // vadstena_libs_registry_freelayer_hpp_included_
