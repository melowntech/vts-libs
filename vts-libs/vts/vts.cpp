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

} } // namespace vadstena::vts
