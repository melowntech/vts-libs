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
 * \file registry/json.hpp
 * \author Vaclav Blazek <vaclav.blazek@citationtech.net>
 */

#ifndef vtslibs_registry_json_hpp_included_
#define vtslibs_registry_json_hpp_included_

#include "jsoncpp/json.hpp"

#include "../registry.hpp"

namespace vtslibs { namespace registry {

Json::Value asJson(const ReferenceFrame &rf);
ReferenceFrame referenceFrame(const Json::Value &value
                              , const boost::filesystem::path &path
                              = "unknown");
void fromJson(ReferenceFrame &referenceFrame, const Json::Value &value
              , const boost::filesystem::path &path
              = "unknown");

Json::Value asJson(const Srs::dict &srs, bool flatGeoidPath = false);
void fromJson(Srs::dict &srs, const Json::Value &value);

Json::Value asJson(const Credits &credits, bool inlineCredits = true);
void fromJson(Credits &credits, const Json::Value &value);

Json::Value asJson(const Body::dict &bodies);
Body::dict bodiesFromJson(const Json::Value &value);

Json::Value asJson(const Credit::dict &credits);
Credit::dict creditsFromJson(const Json::Value &value);
void fromJson(Credit::dict &credits, const Json::Value &value);

Json::Value asJson(const BoundLayer::dict &boundLayers
                   , bool inlineCredits = true);
BoundLayer::dict boundLayersFromJson(const Json::Value &value);
void fromJson(BoundLayer::dict &boundLayers, const Json::Value &value);

Json::Value asJson(const FreeLayer::dict &freeLayers
                   , bool inlineCredits = true);
Json::Value asJson(const FreeLayer &freeLayer
                   , bool inlineCredits = true);
FreeLayer::dict freeLayerFromJson(const Json::Value &value);
void fromJson(FreeLayer::dict &freeLayers, const Json::Value &value);

Json::Value asJson(const Position &position);
Position positionFromJson(const Json::Value &value);
TileRange tileRangeFromJson(const Json::Value &value);

Json::Value asJson(const View &view, BoundLayer::dict &boundLayers);
void fromJson(View &view, const Json::Value &value);
View viewFromJson(const Json::Value &value);

Json::Value asJson(const Roi &roi);
Json::Value asJson(const Roi::list &rois);
Roi::list roisFromJson(const Json::Value &value);
Json::Value asJson(const View::map &namedViews
                   , BoundLayer::dict &boundLayers);
void fromJson(View::map &views, const Json::Value &value);
View::map namedViewsFromJson(const Json::Value &value);

Json::Value asJson(const RegistryBase &r);
void fromJson(RegistryBase &r, const Json::Value &value);

Json::Value asJson(const Service::dict &services);
void fromJson(Service::dict &services, const Json::Value &value);

namespace extensions {

boost::any fromJson(const std::string &key, const Json::Value &value);
Json::Value asJson(const boost::any &value);

} // namespace extensions

// inlines

inline ReferenceFrame referenceFrame(const Json::Value &value
                                     , const boost::filesystem::path &path)
{
    ReferenceFrame rf;
    fromJson(rf, value, path);
    return rf;
}

inline Credit::dict creditsFromJson(const Json::Value &value)
{
    Credit::dict credits;
    fromJson(credits, value);
    return credits;
}

inline BoundLayer::dict boundLayersFromJson(const Json::Value &value)
{
    BoundLayer::dict boundLayers;
    fromJson(boundLayers, value);
    return boundLayers;
}

inline FreeLayer::dict freeLayersFromJson(const Json::Value &value)
{
    FreeLayer::dict freeLayers;
    fromJson(freeLayers, value);
    return freeLayers;
}

inline View viewFromJson(const Json::Value &value)
{
    View view;
    fromJson(view, value);
    return view;
}

inline View::map namedViewsFromJson(const Json::Value &value)
{
    View::map views;
    fromJson(views, value);
    return views;
}

} } // namespace vtslibs::registry

#endif // vtslibs_registry_json_hpp_included_
