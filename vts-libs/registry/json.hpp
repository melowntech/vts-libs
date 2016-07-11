/**
 * \file registry/json.hpp
 * \author Vaclav Blazek <vaclav.blazek@citationtech.net>
 */

#ifndef vadstena_libs_registry_json_hpp_included_
#define vadstena_libs_registry_json_hpp_included_

#include "jsoncpp/json.hpp"

#include "./referenceframe.hpp"

namespace vadstena { namespace registry {

Json::Value asJson(const ReferenceFrame &rf);
ReferenceFrame referenceFrame(const Json::Value &value);
void fromJson(ReferenceFrame &referenceFrame, const Json::Value &value);

Json::Value asJson(const Srs::dict &srs, bool flatGeoidPath = false);
void fromJson(Srs::dict &srs, const Json::Value &value);

Json::Value asJson(const Credit::dict &credits);
Credit::dict creditsFromJson(const Json::Value &value);
void fromJson(Credit::dict &srs, const Json::Value &value);

Json::Value asJson(const BoundLayer::dict &boundLayers
                   , bool inlineCredits = true);
BoundLayer::dict boundLayersFromJson(const Json::Value &value);
void fromJson(BoundLayer::dict &srs, const Json::Value &value);

Json::Value asJson(const Position &position);
Position positionFromJson(const Json::Value &value);
TileRange tileRangeFromJson(const Json::Value &value);

Json::Value asJson(const View &view, BoundLayer::dict &boundLayers);
View viewFromJson(const Json::Value &value);
Json::Value asJson(const Roi &roi);
Json::Value asJson(const Roi::list &rois);
Roi::list roisFromJson(const Json::Value &value);
Json::Value asJson(const View::map &namedViews
                   , BoundLayer::dict &boundLayers);
View::map namedViewsFromJson(const Json::Value &value);

// inlines

inline ReferenceFrame referenceFrame(const Json::Value &value)
{
    ReferenceFrame rf;
    fromJson(rf, value);
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

} } // namespace vadstena::registry

#endif // vadstena_libs_registry_json_hpp_included_
