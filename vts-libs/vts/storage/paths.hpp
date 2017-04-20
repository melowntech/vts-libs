/**
 * Copyright (c) 2017 Melown Technologies SE
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * *  Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
/**
 * \file vts/storage/paths.hpp
 * \author Vaclav Blazek <vaclav.blazek@citationtech.net>
 *
 * Storage paths
 */

#ifndef vtslibs_vts_storage_paths_hpp_included_
#define vtslibs_vts_storage_paths_hpp_included_

#include <boost/optional.hpp>
#include <boost/filesystem/path.hpp>

#include "../storage.hpp"

namespace vtslibs { namespace vts { namespace storage_paths {

/** Get root for storage tilesets.
 */
inline boost::filesystem::path tilesetRoot() { return "tilesets"; }

/** Get root for storage glues.
 */
inline boost::filesystem::path glueRoot() { return "glues"; }

/** Get root for storage virtualSurfaces.
 */
inline boost::filesystem::path virtualSurfaceRoot() { return "vs"; }

/** Get (local) path to glue rules.
 */
inline boost::filesystem::path glueRulesPath() { return "glue.rules"; }

/** Get (local) path to merge configuration.
 */
inline boost::filesystem::path mergeConfPath() { return "merge.conf"; }

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

/** Generate path for storage virtualSurface. If tmp is false regular storage
 *  path is generated.  Otherwise root / "tmp" is used unless different tmpRoot
 *  is provided.
 */
boost::filesystem::path
virtualSurfacePath(const boost::filesystem::path &root
                   , const VirtualSurface &virtualSurface
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

inline boost::filesystem::path
virtualSurfacePath(const boost::filesystem::path &root
                   , const VirtualSurface &virtualSurface
                   , bool tmp
                   , const boost::optional<boost::filesystem::path> &tmpRoot)
{
    if (tmp) {
        if (tmpRoot) {
            return *tmpRoot / virtualSurface.path;
        }
        return root / "tmp" / virtualSurface.path;
    }

    return root / virtualSurfaceRoot() / virtualSurface.path;
}

} } } // namespace vtslibs::vts::storage_paths

#endif // vtslibs_vts_storage_paths_hpp_included_
