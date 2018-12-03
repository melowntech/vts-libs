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

#include <boost/algorithm/string/predicate.hpp>

#include <boost/filesystem.hpp>

#include "../vts.hpp"

namespace ba = boost::algorithm;
namespace fs = boost::filesystem;

namespace vtslibs { namespace vts {

DatasetType datasetType(const boost::filesystem::path &path)
{
    if (TileSet::check(path)) {
        return DatasetType::TileSet;
    } else if (Storage::check(path)) {
        return DatasetType::Storage;
    } else if (StorageView::check(path)) {
        return DatasetType::StorageView;
    } else if (TileIndex::check(path)) {
        return DatasetType::TileIndex;
    }
    return DatasetType::Unknown;
}

DatasetType datasetType(const boost::filesystem::path &path
                        , const std::string &mime)
{
    if (TileSet::check(path, mime)) {
        return DatasetType::TileSet;
    } else if (Storage::check(path, mime)) {
        return DatasetType::Storage;
    } else if (StorageView::check(path, mime)) {
        return DatasetType::StorageView;
    } else if (TileIndex::check(path, mime)) {
        return DatasetType::TileIndex;
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

TileSet aggregateTileSets(const boost::filesystem::path &path
                          , const Storage &storage
                          , const CloneOptions &createOptions
                          , const TilesetIdSet &tilesets)
{
    return aggregateTileSets(path, storage.path(), createOptions, tilesets);
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

} } // namespace vtslibs::vts
