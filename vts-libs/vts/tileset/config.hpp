/**
 * \file vts/config.hpp
 * \author Vaclav Blazek <vaclav.blazek@citationtech.net>
 */

#ifndef vadstena_libs_vts_tileset_config_hpp_included_
#define vadstena_libs_vts_tileset_config_hpp_included_

#include <iostream>

#include <boost/filesystem/path.hpp>

#include "../tileset.hpp"

namespace vadstena { namespace vts {

TileSet::Properties loadConfig(std::istream &in);

void saveConfig(std::ostream &out, const TileSet::Properties &properties);

TileSet::Properties loadConfig(const boost::filesystem::path &path);

void saveConfig(const boost::filesystem::path &path
                , const TileSet::Properties &properties);

} } // namespace vadstena::vts

#endif // vadstena_libs_vts_tileset_config_hpp_included_
