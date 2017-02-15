/**
 * \file vts/config.hpp
 * \author Vaclav Blazek <vaclav.blazek@citationtech.net>
 */

#ifndef vtslibs_vts_storageview_config_hpp_included_
#define vtslibs_vts_storageview_config_hpp_included_

#include <iostream>

#include <boost/filesystem/path.hpp>

#include "../storageview.hpp"

namespace vtslibs { namespace vts { namespace storageview {

StorageView::Properties loadConfig(std::istream &in);

StorageView::Properties loadConfig(const boost::filesystem::path &path);

void saveConfig(const boost::filesystem::path &path
                , const StorageView::Properties &properties);

bool checkConfig(std::istream &in);

bool checkConfig(const boost::filesystem::path &path);

} } } // namespace vtslibs::vts::storageview

#endif // vtslibs_vts_storageview_config_hpp_included_
