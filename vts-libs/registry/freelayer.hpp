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
 * \file registry/freelayer.hpp
 * \author Vaclav Blazek <vaclav.blazek@citationtech.net>
 */

#ifndef vtslibs_registry_freelayer_hpp_included_
#define vtslibs_registry_freelayer_hpp_included_

#include <set>
#include <map>
#include <string>
#include <vector>
#include <new>

#include <boost/variant.hpp>

#include "utility/uri.hpp"

#include "referenceframe.hpp"

namespace vtslibs { namespace registry {

struct FreeLayer {
    static constexpr char typeName[] = "free layer";

    enum class Type { external, geodata, geodataTiles, meshTiles };

    struct Geodata {
        math::Extents3 extents;
        int displaySize;
        std::string label;
        std::string geodata;
        std::string style;
        boost::any options;

        Geodata() : displaySize() {}
    };

    struct GeodataTiles {
        LodRange lodRange;
        TileRange tileRange;
        int displaySize;
        std::string metaUrl;
        std::string geodataUrl;
        std::string style;
        boost::any options;

        GeodataTiles() : displaySize() {}
    };

    struct MeshTiles {
        LodRange lodRange;
        TileRange tileRange;
        std::string metaUrl;
        std::string meshUrl;
        std::string textureUrl;
        boost::any options;
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

    bool external() const { return type == Type::external; }

    const std::string& externalUrl() const {
        return boost::get<std::string>(definition);
    }
};

void saveFreeLayer(std::ostream &out, const FreeLayer &freeLayer);

FreeLayer loadFreeLayer(std::istream &in
                        , const boost::filesystem::path &path = "unknown");

/** Make all URL's absolute.
 */
FreeLayer absolutize(const FreeLayer &freeLayer, const utility::Uri &baseUrl);

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

} } // namespace vtslibs::registry

#endif // vtslibs_registry_freelayer_hpp_included_
