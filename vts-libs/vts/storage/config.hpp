/**
 * \file vts/config.hpp
 * \author Vaclav Blazek <vaclav.blazek@citationtech.net>
 */

#ifndef vadstena_libs_vts_storage_config_hpp_included_
#define vadstena_libs_vts_storage_config_hpp_included_

#include <iostream>

#include <boost/filesystem/path.hpp>

#include "../storage.hpp"

namespace Json { class Value; }

namespace vadstena { namespace vts { namespace storage {

Storage::Properties loadConfig(std::istream &in);

void saveConfig(std::ostream &out, const Storage::Properties &properties);

Storage::Properties loadConfig(const boost::filesystem::path &path);

void saveConfig(const boost::filesystem::path &path
                , const Storage::Properties &properties);

ExtraStorageProperties loadExtraConfig(std::istream &in);

ExtraStorageProperties loadExtraConfig(const boost::filesystem::path &path);

ExtraStorageProperties
extraStorageConfigFromJson(int version, const Json::Value &config);

void extraStorageConfigToJson(Json::Value &config
                              , const ExtraStorageProperties &properties);

} } } // namespace vadstena::vts::storage

#endif // vadstena_libs_vts_storage_config_hpp_included_
