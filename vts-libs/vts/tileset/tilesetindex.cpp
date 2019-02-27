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
#include "utility/streams.hpp"

#include "driver.hpp"
#include "tilesetindex.hpp"

namespace fs = boost::filesystem;

namespace vtslibs { namespace vts { namespace tileset {

void loadTileSetIndex(Index &tsi, const Driver &driver)
{
    try {
        tsi.tileIndex = {};
        auto f(driver.input(File::tileIndex));
        tsi.tileIndex.load(*f, f->name());

        // load rest of data
        tsi.loadRest(*f, f->name());

        f->close();
    } catch (const std::exception &e) {
        LOGTHROW(err2, storage::Error)
            << "Unable to read tile index: " << e.what() << ".";
    }
}

Index::pointer loadTileSetIndex(const Driver &driver)
{
    auto tsi(std::make_shared<Index>());
    loadTileSetIndex(*tsi, driver);
    return tsi;
}

void saveTileSetIndex(const Index &tsi, Driver &driver)
{
    try {
        auto f(driver.output(File::tileIndex));
        tsi.tileIndex.save(*f);
        tsi.saveRest(*f);
        f->close();
    } catch (const std::exception &e) {
        LOGTHROW(err2, storage::Error)
            << "Unable to save tile index: " << e.what() << ".";
    }
}

void loadTileSetIndex(Index &tsi, const fs::path &path)
{
    try {
        utility::ifstreambuf f(path.c_str());

        tsi.tileIndex = {};
        tsi.tileIndex.load(f, path);

        tsi.loadRest(f, path);
        f.close();
    } catch (const std::exception &e) {
        LOGTHROW(err1, storage::Error)
            << "Unable to read tile index: " << e.what() << ".";
    }
}

void saveTileSetIndex(const Index &tsi, const fs::path &path)
{
    try {
        utility::ofstreambuf f(path.c_str());
        tsi.tileIndex.save(f);
        tsi.saveRest(f);
        f.close();
    } catch (const std::exception &e) {
        LOGTHROW(err1, storage::Error)
            << "Unable to save tile index: " << e.what() << ".";
    }
}

void saveTileSetIndex(const Index &tsi, std::ostream &os)
{
    try {
        tsi.tileIndex.save(os);
        tsi.saveRest(os);
    } catch (const std::exception &e) {
        LOGTHROW(err1, storage::Error)
            << "Unable to save tile index: " << e.what() << ".";
    }
}

bool Index::meta(const TileId &tileId) const
{
    return tileIndex.validSubtree
        (tileId.lod, parent(tileId, metaBinaryOrder_));
}

bool Index::check(const TileId &tileId, TileFile type) const
{
    switch (type) {
    case TileFile::meta:
        return meta(tileId);
    case TileFile::mesh:
        return tileIndex.checkMask(tileId, TileIndex::Flag::mesh);
    case TileFile::atlas:
        return tileIndex.checkMask(tileId, TileIndex::Flag::atlas);
    case TileFile::navtile:
        return tileIndex.checkMask(tileId, TileIndex::Flag::navtile);

    default: return false;
    }
}

TileIndex::Flag::value_type
Index::checkAndGetFlags(const TileId &tileId, TileFile type) const
{
    const auto flags(tileIndex.get(tileId));

    switch (type) {
    case TileFile::mesh:
        return (flags & TileIndex::Flag::mesh) ? flags : 0;
    case TileFile::atlas:
        return (flags & TileIndex::Flag::atlas) ? flags : 0;
    case TileFile::navtile:
        return (flags & TileIndex::Flag::navtile) ? flags : 0;

    default: return 0;
    }
}

TileIndex Index::deriveMetaIndex(bool contentOnly) const
{
    if (tileIndex.empty()) { return {}; }

    // make tileindex copy
    TileIndex ti(tileIndex);
    if (contentOnly) {
        // filter-out non-content tiles
        ti.simplify(TileIndex::Flag::content);
    }
    return ti.shrinkAndComplete(metaBinaryOrder_);
}

TileIndex Index::deriveMetaIndex(Lod ceiling, bool contentOnly) const
{
    if (tileIndex.empty()) { return {}; }

    // make tileindex copy
    TileIndex ti(LodRange(ceiling, tileIndex.maxLod()), &tileIndex, false);
    if (contentOnly) {
        // filter-out non-content tiles
        ti.simplify(TileIndex::Flag::content);
    }
    return ti.shrinkAndComplete(metaBinaryOrder_);
}

void Index::loadRest_impl(std::istream&, const boost::filesystem::path&) {}

void Index::saveRest_impl(std::ostream&) const {}

} } } // namespace vtslibs::vts::tileset
