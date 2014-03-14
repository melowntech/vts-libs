#ifndef vadstena_libs_tilestorage_traverser_hpp_included_
#define vadstena_libs_tilestorage_traverser_hpp_included_

#include "./basetypes.hpp"
#include "./maskfwd.hpp"

namespace vadstena { namespace tilestorage {

class TileIndex;

class Traverser {
public:
    Traverser(const TileIndex *owner);

    struct Tile {
        Tile(const Index &index, const TileId &tileId, bool value)
            : index(index), id(tileId), value(value), valid_(true)
        {}
        Tile() : value(false), valid_(false) {}

        Index index;
        TileId id;
        bool value;

        operator bool() const { return valid_; }

    private:
        bool valid_;
    };

    Tile next();

private:
    void load();

    const TileIndex *owner_;
    const RasterMask *mask_;
    Size2l size_;
    Index index_;
};

} } // namespace vadstena::tilestorage

#endif // vadstena_libs_tilestorage_traverser_hpp_included_
