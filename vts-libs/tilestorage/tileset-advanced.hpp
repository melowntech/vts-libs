/**
 * \file tilestorage/tileset.hpp
 * \author Vaclav Blazek <vaclav.blazek@citationtech.net>
 *
 * Advanced tile set API.
 *
 */

#ifndef vadstena_libs_tilestorage_tileset_advanced_hpp_included_
#define vadstena_libs_tilestorage_tileset_advanced_hpp_included_

#include "./tileset.hpp"
#include "./streams.hpp"

namespace vadstena { namespace tilestorage {

/** TileSet advanced interface.
 */
class TileSet::AdvancedApi {
public:
    // creates new api
    AdvancedApi(const TileSet::pointer &tileSet) : tileSet_(tileSet) {}

    /** Returns traverser over tiles.
     */
    Traverser tileTraverser() const;

    /** Returns traverser over metatiles.
     */
    Traverser metaTraverser() const;

    /** Returns output stream for given file.
     */
    OStream::pointer output(File type);

    /** Returns input stream for given file.
     */
    IStream::pointer input(File type) const;

    /** Returns output stream for given tile file.
     */
    OStream::pointer output(const TileId tileId, TileFile type);

    /** Returns input stream for given tile file.
     */
    IStream::pointer input(const TileId tileId, TileFile type) const;

    /** Change metalevels. Metatiles and tile/meta indices are regenerated.
     */
    void changeMetaLevels(const LodLevels &metaLevels);

    /** Renames tile set, i.e. its id is changed.
     */
    void rename(const std::string &newId);

    /** Regenerate index from metadata.
     */
    void regenerateTileIndex();

private:
    TileSet::pointer tileSet_;
};

} } // namespace vadstena::tilestorage

#endif // vadstena_libs_tilestorage_tileset_advanced_hpp_included_
