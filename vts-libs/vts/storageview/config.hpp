/**
 * \file vts/config.hpp
 * \author Vaclav Blazek <vaclav.blazek@citationtech.net>
 */

#ifndef vadstena_libs_vts_storageview_config_hpp_included_
#define vadstena_libs_vts_storageview_config_hpp_included_

#include <iostream>

#include <boost/filesystem/path.hpp>

#include "../storageview.hpp"

namespace vadstena { namespace vts { namespace storageview {

StorageView::Properties loadConfig(std::istream &in);

StorageView::Properties loadConfig(const boost::filesystem::path &path);

} } } // namespace vadstena::vts::storageview

#endif // vadstena_libs_vts_storageview_config_hpp_included_
