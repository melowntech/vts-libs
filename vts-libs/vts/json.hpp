/**
 * \file vts/json.hpp
 * \author Vaclav Blazek <vaclav.blazek@citationtech.net>
 */

#ifndef vadstena_libs_vts_json_hpp_included_
#define vadstena_libs_vts_json_hpp_included_

#include "jsoncpp/json.hpp"

#include "mapconfig.hpp"

namespace vadstena { namespace vts {

/** Wrapper around value.
 */
struct BrowserCoreOptions {
    Json::Value value;

    BrowserCoreOptions(const Json::Value &value) : value(value) {}
};

} } // namespace vadstena::vts

#endif // vadstena_libs_vts_mapconfig_hpp_included_
