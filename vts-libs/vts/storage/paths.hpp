/**
 * \file vts/storage/paths.hpp
 * \author Vaclav Blazek <vaclav.blazek@citationtech.net>
 *
 * Storage paths
 */

#ifndef vadstena_libs_vts_storage_paths_hpp_included_
#define vadstena_libs_vts_storage_paths_hpp_included_

#include <boost/optional.hpp>
#include <boost/filesystem/path.hpp>

#include "../storage.hpp"

namespace vadstena { namespace vts { namespace storage_paths {

/** Get root for storage tilesets.
 */
inline boost::filesystem::path tilesetRoot() { return "tilesets"; }

/** Get root for storage glues.
 */
inline boost::filesystem::path glueRoot() { return "glues"; }

/** Generate path for storage tileset. If tmp is false regular storage path is
 *  generated.
 *  Otherwise root / "tmp" is used unless different tmpRoot is provided.
 */
boost::filesystem::path
tilesetPath(const boost::filesystem::path &root, const std::string &tilesetId
            , bool tmp = false
            , const boost::optional<boost::filesystem::path> &tmpRoot
            = boost::none);

/** Generate path for storage glue. If tmp is false regular storage path is
 *  generated.
 *  Otherwise root / "tmp" is used unless different tmpRoot is provided.
 */
boost::filesystem::path
gluePath(const boost::filesystem::path &root, const Glue &glue
         , bool tmp = false
         , const boost::optional<boost::filesystem::path> &tmpRoot
         = boost::none);

// inlines

inline boost::filesystem::path
tilesetPath(const boost::filesystem::path &root, const std::string &tilesetId
            , bool tmp
            , const boost::optional<boost::filesystem::path> &tmpRoot)
{
    if (tmp) {
        if (tmpRoot) {
            return *tmpRoot / tilesetId;
        }
        return root / "tmp" / tilesetId;
    }

    return root / tilesetRoot() / tilesetId;
}

inline boost::filesystem::path
gluePath(const boost::filesystem::path &root, const Glue &glue
         , bool tmp
         , const boost::optional<boost::filesystem::path> &tmpRoot)
{
    if (tmp) {
        if (tmpRoot) {
            return *tmpRoot / glue.path;
        }
        return root / "tmp" / glue.path;
    }

    return root / glueRoot() / glue.path;
}

} } } // namespace vadstena::vts::storage_paths

#endif // vadstena_libs_vts_storage_paths_hpp_included_
