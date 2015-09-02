#ifndef vadstena_libs_vts0_json_hpp_included_
#define vadstena_libs_vts0_json_hpp_included_

#include "jsoncpp/json.hpp"

#include "./properties.hpp"
#include "./storage.hpp"

namespace vadstena { namespace vts0 {

void parse(Properties &properties, const Json::Value &config);

void build(Json::Value &config, const Properties &properties);

void parse(StorageProperties &properties, const Json::Value &config);

void build(Json::Value &config, const StorageProperties &properties);

} } // namespace vadstena::vts0

#endif // vadstena_libs_vts0_json_hpp_included_
