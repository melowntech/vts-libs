/**
 * \file vts/config.hpp
 * \author Vaclav Blazek <vaclav.blazek@citationtech.net>
 */

#ifndef vadstena_libs_vts_tileset_config_hpp_included_
#define vadstena_libs_vts_tileset_config_hpp_included_

#include <iostream>

#include <boost/any.hpp>
#include <boost/optional.hpp>
#include <boost/filesystem/path.hpp>

#include "../tileset.hpp"

namespace vadstena { namespace vts { namespace tileset {

FullTileSetProperties loadConfig(std::istream &in
                                 , const boost::filesystem::path &path
                                 = "unknown");

void saveConfig(std::ostream &out, const FullTileSetProperties &properties);

FullTileSetProperties loadConfig(const boost::filesystem::path &path);

void saveConfig(const boost::filesystem::path &path
                , const FullTileSetProperties &properties);

ExtraTileSetProperties loadExtraConfig(std::istream &in);

ExtraTileSetProperties loadExtraConfig(const boost::filesystem::path &path);

boost::optional<unsigned int> loadRevision(std::istream &in);

boost::optional<unsigned int>
loadRevision(const boost::filesystem::path &path);

FullTileSetProperties loadConfig(const Driver &driver);

FullTileSetProperties loadConfig(const IStream::pointer &file);

// bare driver support

void saveDriver(std::ostream &out, const boost::any &driver);

void saveDriver(const boost::filesystem::path &path
                , const boost::any &driver);

boost::any loadDriver(std::istream &in, const boost::filesystem::path &path
                      = "unknown");

boost::any loadDriver(const boost::filesystem::path &path);


} } } // namespace vadstena::vts::tileset

#endif // vadstena_libs_vts_tileset_config_hpp_included_
