#include <boost/algorithm/string/predicate.hpp>

#include <boost/filesystem.hpp>

#include "../vts.hpp"

namespace ba = boost::algorithm;
namespace fs = boost::filesystem;

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

/** Creates in-memory aggreagated tileset from storage subset.
 */
TileSet aggregateTileSets(const Storage &storage
                          , const CloneOptions &co
                          , const TilesetIdList &tilesets)
{
    return aggregateTileSets(storage, co
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

std::istream& operator>>(std::istream &is, RelocateOptions::Rule &rule)
{
    bool prefix(true);
    std::string *out(&rule.prefix);
    for (;;) {
        auto c = is.peek();
        if (is.eof()) { break; }
        c = is.get();

        if (prefix && (c == '=')) {
            // prefix matched
            out = &rule.replacement;
            continue;
        }
        // copy output
        out->push_back(c);
    }
    return is;
}

RelocateOptions::Result RelocateOptions::apply(const std::string &path) const
{
    // default to follow path
    Result result(path);

    for (const auto &rule : rules) {
        if (!ba::istarts_with(path, rule.prefix)) { continue; }

        // match, join replacement with rest of path and we're done here
        result.replacement
            = (rule.replacement + path.substr(rule.prefix.size()));
        break;
    }

    // original path exists -> follow it
    if (fs::exists(path)) {
        return result;
    }

    // otherwise if replacement exists -> follow it
    if (result.replacement && fs::exists(*result.replacement)) {
        result.follow = *result.replacement;
        return result;
    }

    // no path to follow, return result as is
    return result;
}

} } // namespace vadstena::vts
