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
 * \file vts/storageview.hpp
 * \author Vaclav Blazek <vaclav.blazek@citationtech.net>
 *
 * Tile set storage view access.
 */

#ifndef vtslibs_vts_storageview_hpp_included_
#define vtslibs_vts_storageview_hpp_included_

#include <memory>
#include <string>

#include <boost/optional.hpp>
#include <boost/noncopyable.hpp>
#include <boost/filesystem/path.hpp>

#include "utility/enum-io.hpp"

#include "tileset.hpp"
#include "basetypes.hpp"

#include "storage.hpp"

namespace vtslibs { namespace vts {

/** Properties, imports extra storage properties.
 */
struct StorageViewProperties  {
    /** Path to storage. Can be relative.
     */
    boost::filesystem::path storagePath;

    /** Set of tilesets.
     */
    TilesetIdSet tilesets;

    /** Set of tilesets returned as free surfaces.
     */
    TilesetIdSet freeLayerTilesets;

    /** Extra properties.
     */
    ExtraStorageProperties extra;

    /** Absolutizes all filesystem paths.
     */
    void absolutize(const boost::filesystem::path &root);

    typedef std::function<void(ExtraStorageProperties&)> ExtraFilter;
};


/** StorageView interface.
 */
class StorageView {
public:
    /** Internal properties. Defined in detail.
     */
    struct Properties;

    /** Opens storage view.
     */
    StorageView(const boost::filesystem::path &path);

    /** Opens storage view. Support for async open.
     */
    StorageView(const boost::filesystem::path &path
                , const Properties &properties, Storage storage);

    ~StorageView();

    bool externallyChanged() const;

    std::time_t lastModified() const;

    vtslibs::storage::Resources resources() const;

    /** Generates map configuration for this storage view.
     */
    MapConfig mapConfig() const;

    const Storage& storage() const;

    const TilesetIdSet& tilesets() const;

    boost::filesystem::path storagePath() const;

    /** Flattens content of this storageview into new tileset at tilesetPath.
     *
     * \param tilesetPath path to tileset
     * \param createOptions create options
     */
    TileSet clone(const boost::filesystem::path &tilesetPath
                  , const CloneOptions &createOptions);

    /** Get list of pending glues needed to display this storage view.
     */
    Glue::IdSet pendingGlues() const;

    /** Saves configuration to another path.
     *  Provided filter can manipulate with extra storage propeties.
     */
    void saveConfig(const boost::filesystem::path &path
                    , const StorageViewProperties::ExtraFilter &extraFilter
                    = StorageViewProperties::ExtraFilter()
                    , bool relativePaths = false);

    /** Generates map configuration for storage view at given path.
     */
    static MapConfig mapConfig(const boost::filesystem::path &path);

    /** Check for storageview at given path.
     */
    static bool check(const boost::filesystem::path &path);

    static bool check(const boost::filesystem::path &path
                      , const std::string &mime);

    static void relocate(const boost::filesystem::path &root
                         , const RelocateOptions &options
                         , const std::string &prefix = "");

    static void reencode(const boost::filesystem::path &root
                         , const ReencodeOptions &options
                         , const std::string &prefix = "");

    /** Internals. Public to ease library developers' life, not to allow users
     *  to put their dirty hands in the storageview's guts!
     */
    struct Detail;

private:
    std::shared_ptr<Detail> detail_;
    Detail& detail() { return *detail_; }
    const Detail& detail() const { return *detail_; }
};


} } // namespace vtslibs::vts

#endif // vtslibs_vts_storageview_hpp_included_
