#ifndef vtslibs_tilestorage_json_hpp_included_
#define vtslibs_tilestorage_json_hpp_included_

#include "jsoncpp/json.hpp"

#include "./properties.hpp"
#include "./storage.hpp"

namespace vtslibs { namespace tilestorage {

void parse(Properties &properties, const Json::Value &config);

void build(Json::Value &config, const Properties &properties);

void parse(StorageProperties &properties, const Json::Value &config);

void build(Json::Value &config, const StorageProperties &properties);

} } // namespace vtslibs::tilestorage

#endif // vtslibs_tilestorage_json_hpp_included_
