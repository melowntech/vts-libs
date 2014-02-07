#ifndef vadstena_libs_tilestorage_json_hpp_included_
#define vadstena_libs_tilestorage_json_hpp_included_

#include "jsoncpp/json.hpp"

#include "./tileset.hpp"
#include "./storage.hpp"

namespace vadstena { namespace tilestorage {

void parse(Properties &properties, const Json::Value &config);

void build(Json::Value &config, const Properties &properties);

void parse(StorageProperties &properties, const Json::Value &config);

void build(Json::Value &config, const StorageProperties &properties);

} } // namespace vadstena::tilestorage

#endif // vadstena_libs_tilestorage_json_hpp_included_
