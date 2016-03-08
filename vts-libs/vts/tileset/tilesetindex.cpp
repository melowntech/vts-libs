#include "./tilesetindex.hpp"

namespace vadstena { namespace vts { namespace tileset {

void loadTileSetIndex(Index &tsi, const Driver &driver)
{
    try {
        tsi.tileIndex = {};
        auto f(driver.input(File::tileIndex));
        tsi.tileIndex.load(*f, f->name());

        if (f->get().peek() != std::istream::traits_type::eof()) {
            tsi.references.load(*f, f->name());
        }
        f->close();
    } catch (const std::exception &e) {
        LOGTHROW(err2, storage::Error)
            << "Unable to read tile index: " << e.what() << ".";
    }
}

void saveTileSetIndex(const Index &tsi, Driver &driver)
{
    try {
        auto f(driver.output(File::tileIndex));
        tsi.tileIndex.save(*f);
        tsi.references.save(*f);
        f->close();
    } catch (const std::exception &e) {
        LOGTHROW(err2, storage::Error)
            << "Unable to save tile index: " << e.what() << ".";
    }
}

bool Index::check(const TileId &tileId, TileFile type) const
{
    switch (type) {
    case TileFile::meta:
        return tileIndex.checkMask(tileId, TileIndex::Flag::meta);
    case TileFile::mesh:
        return tileIndex.checkMask(tileId, TileIndex::Flag::mesh);
    case TileFile::atlas:
        return tileIndex.checkMask(tileId, TileIndex::Flag::atlas);
    case TileFile::navtile:
        return tileIndex.checkMask(tileId, TileIndex::Flag::navtile);

    default: return false;
    }
}

int Index::getReference(const TileId &tileId) const
{
    if (!tileIndex.checkMask(tileId, TileIndex::Flag::reference)) {
        return 0;
    }
    return references.get(tileId);
}

} } } // namespace vadstena::vts::tileset
