#include "../vts.hpp"

namespace vadstena { namespace vts {

DatasetType datasetType(const boost::filesystem::path &path)
{
    if (TileSet::check(path)) {
        return DatasetType::TileSet;
    } else if (Storage::check(path)) {
        return DatasetType::Storage;
    } else if (StorageView::check(path)) {
        return DatasetType::StorageView;
    }

    return DatasetType::Unknown;
}

/** Creates aggreagated tileset from storage subset.
 */
TileSet aggregateTileSets(const boost::filesystem::path &path
                          , const Storage &storage
                          , const CloneOptions &co
                          , const TilesetIdList &tilesets)
{
    return aggregateTileSets(path, storage, co
                             , TilesetIdSet(tilesets.begin(), tilesets.end()));
}

/** Creates aggreagated tileset from storage view.
 */
TileSet aggregateTileSets(const boost::filesystem::path &path
                          , const StorageView &storageView
                          , const CloneOptions &co)
{
    return aggregateTileSets(path, storageView.storage(), co
                             , storageView.tilesets());
}

} } // namespace vadstena::vts
