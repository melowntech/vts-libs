/**
 * \file tilestorage/config.hpp
 * \author Vaclav Blazek <vaclav.blazek@citationtech.net>
 */

#ifndef vtslibs_tilestorage_config_hpp_included_
#define vtslibs_tilestorage_config_hpp_included_

#include <iostream>

#include <boost/filesystem/path.hpp>

#include "./properties.hpp"

namespace vtslibs { namespace tilestorage {

Properties loadConfig(std::istream &in);

void saveConfig(std::ostream &out, const Properties &properties);

Properties loadConfig(const boost::filesystem::path &path);

void saveConfig(const boost::filesystem::path &path
                , const Properties &properties);

} } // namespace vtslibs::tilestorage

#endif // vtslibs_tilestorage_config_hpp_included_
