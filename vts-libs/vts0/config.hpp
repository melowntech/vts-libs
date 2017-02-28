/**
 * \file vts/config.hpp
 * \author Vaclav Blazek <vaclav.blazek@citationtech.net>
 */

#ifndef vtslibs_vts0_config_hpp_included_
#define vtslibs_vts0_config_hpp_included_

#include <iostream>

#include <boost/filesystem/path.hpp>

#include "./properties.hpp"

namespace vtslibs { namespace vts0 {

Properties loadConfig(std::istream &in);

void saveConfig(std::ostream &out, const Properties &properties);

Properties loadConfig(const boost::filesystem::path &path);

void saveConfig(const boost::filesystem::path &path
                , const Properties &properties);

} } // namespace vtslibs::vts0

#endif // vtslibs_vts0_config_hpp_included_
