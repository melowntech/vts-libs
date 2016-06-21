/**
 * \file vts/storageview.hpp
 * \author Vaclav Blazek <vaclav.blazek@citationtech.net>
 *
 * Tile set storage view access.
 */

#ifndef vadstena_libs_vts_storageview_hpp_included_
#define vadstena_libs_vts_storageview_hpp_included_

#include <memory>
#include <string>

#include <boost/optional.hpp>
#include <boost/noncopyable.hpp>
#include <boost/filesystem/path.hpp>

#include "utility/enum-io.hpp"

#include "./tileset.hpp"
#include "./basetypes.hpp"

#include "./storage.hpp"

namespace vadstena { namespace vts {

/** Properties, imports extra storage properties.
 */
struct StorageViewProperties  {
    /** Path to storage. Can be relative.
     */
    boost::filesystem::path storagePath;

    /** Set of tilesets.
     */
    TilesetIdSet tilesets;

    /** Extra properties.
     */
    ExtraStorageProperties extra;
};

/** StorageView interface.
 */
class StorageView {
public:
    /** Opens storage view.
     */
    StorageView(const boost::filesystem::path &path);

    ~StorageView();

    bool externallyChanged() const;

    std::time_t lastModified() const;

    vadstena::storage::Resources resources() const;

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

    /** Generates map configuration for storage view at given path.
     */
    static MapConfig mapConfig(const boost::filesystem::path &path);

    /** Check for storageview at given path.
     */
    static bool check(const boost::filesystem::path &path);

    /** Internals. Public to ease library developers' life, not to allow users
     *  to put their dirty hands in the storageview's guts!
     */
    struct Detail;

private:
    struct DetailDeleter { void operator()(Detail*); };
    std::unique_ptr<Detail, DetailDeleter> detail_;
    Detail& detail() { return *detail_; }
    const Detail& detail() const { return *detail_; }

public:
    struct Properties;
};


} } // namespace vadstena::vts

#endif // vadstena_libs_vts_storageview_hpp_included_
