/**
 * \file vts/storage/mergeconf.hpp
 * \author Vaclav Blazek <vaclav.blazek@citationtech.net>
 *
 * Merge configuration
 */

#ifndef vtslibs_vts_storage_mergeconf_hpp_included_
#define vtslibs_vts_storage_mergeconf_hpp_included_

#include <memory>

#include "../storage.hpp"

namespace vtslibs { namespace vts {

struct MergeConf {
    OpenOptions::CNames cnames;

    MergeConf() {}
};

/** Load merge config gile.
 *
 * \param path path to merge.conf
 * \param ignoreNoexistent ignore non-existent file (return empty list)
 * \return merge configuration
 */
MergeConf loadMergeConf(const boost::filesystem::path &path
                        , bool ignoreNoexistent = false);

} } // namespace vtslibs::vts

#endif // vtslibs_vts_storage_mergeconf_hpp_included_
