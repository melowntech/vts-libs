/**
 * \file vts/config.hpp
 * \author Vaclav Blazek <vaclav.blazek@citationtech.net>
 */

#ifndef vadstena_libs_vts_tileset_config_hpp_included_
#define vadstena_libs_vts_tileset_config_hpp_included_

#include <iostream>

#include <boost/optional.hpp>
#include <boost/filesystem/path.hpp>

#include "../tileset.hpp"

namespace vadstena { namespace vts { namespace tileset {

TileSet::Properties loadConfig(std::istream &in
                               , const boost::filesystem::path &path
                               = "unknown");

void saveConfig(std::ostream &out, const TileSet::Properties &properties);

TileSet::Properties loadConfig(const boost::filesystem::path &path);

void saveConfig(const boost::filesystem::path &path
                , const TileSet::Properties &properties);

ExtraTileSetProperties loadExtraConfig(std::istream &in);

ExtraTileSetProperties loadExtraConfig(const boost::filesystem::path &path);

boost::optional<unsigned int> loadRevision(std::istream &in);

boost::optional<unsigned int>
loadRevision(const boost::filesystem::path &path);

} } } // namespace vadstena::vts::tileset

#endif // vadstena_libs_vts_tileset_config_hpp_included_
