/**
 * \file vts/storage/detail.hpp
 * \author Vaclav Blazek <vaclav.blazek@citationtech.net>
 *
 * Storage access (internals).
 */

#ifndef vtslibs_vts_storage_detail_hpp_included_
#define vtslibs_vts_storage_detail_hpp_included_

#include <boost/filesystem/path.hpp>

#include "../storageview.hpp"
#include "../storage.hpp"

namespace vtslibs { namespace vts {

using vtslibs::storage::FileStat;

struct StorageView::Properties : StorageViewProperties {
    // nothing so far
};

struct StorageView::Detail
{
    boost::filesystem::path configPath;

    Properties properties;

    registry::ReferenceFrame referenceFrame;

    /** Information about config when tileset was opened.
     */
    FileStat configStat;

    /** Time of last modification
     */
    std::time_t lastModified;

    /** Associated storage.
     */
    Storage storage;

    Detail(const boost::filesystem::path &root);

    ~Detail();

    void loadConfig();

    static StorageView::Properties
    loadConfig(const boost::filesystem::path &configPath);

    void saveConfig();

    bool externallyChanged() const;

    MapConfig mapConfig() const;

    static MapConfig mapConfig(const boost::filesystem::path &configPath);

    static MapConfig mapConfig(const boost::filesystem::path &configPath
                               , const StorageView::Properties &properties);
};

} } // namespace vtslibs::vts

#endif // vtslibs_vts_storage_detail_hpp_included_
