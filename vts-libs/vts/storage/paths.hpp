/**
 * \file vts/storage/paths.hpp
 * \author Vaclav Blazek <vaclav.blazek@citationtech.net>
 *
 * Storage paths
 */

#ifndef vadstena_libs_vts_storage_paths_hpp_included_
#define vadstena_libs_vts_storage_paths_hpp_included_

#include <boost/filesystem/path.hpp>

#include "../storage.hpp"

namespace vadstena { namespace vts { namespace storage_paths {

inline boost::filesystem::path tilesetRoot()
{
    return "tilesets";
}

inline boost::filesystem::path glueRoot()
{
    return "glues";
}

inline boost::filesystem::path
tilesetPath(const boost::filesystem::path &root, const std::string &tilesetId
            , bool tmp = false)
{
    return root / (tmp ? "tmp" : tilesetRoot()) / tilesetId;
}

inline boost::filesystem::path
gluePath(const boost::filesystem::path &root, const Glue &glue
         , bool tmp = false)
{
    return root / (tmp ? "tmp" : glueRoot()) / glue.path;
}

} } } // namespace vadstena::vts::storage_paths

#endif // vadstena_libs_vts_storage_paths_hpp_included_
