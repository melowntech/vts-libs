#ifndef vadstena_libs_tilestorage_tileop_hpp_included_
#define vadstena_libs_tilestorage_tileop_hpp_included_

#include "../tilestorage.hpp"

namespace vadstena { namespace tilestorage {

bool operator==(const TileId &lhs, const TileId &rhs);

bool operator!=(const TileId &lhs, const TileId &rhs);

long tileSize(long baseTileSize, Lod lod);

long tileSize(const Properties &properties, Lod lod);

Extents tileExtents(const Properties &properties, const TileId &tile);

TileId fromAlignment(const Properties &properties, const TileId &tileId);

TileId parent(const Alignment &alignment, long baseTileSize
              , const TileId &tileId);

TileIdChildren children(long baseTileSize, const TileId &tileId);

Lod deltaDown(const LodLevels &levels, Lod lod);

// inline stuff

inline long tileSize(long baseTileSize, Lod lod)
{
    return (baseTileSize >> lod);
}

inline long tileSize(const Properties &properties, Lod lod)
{
    return (properties.baseTileSize >> lod);
}

inline Extents tileExtents(const Properties &properties, const TileId &tile)
{
    auto ts(tileSize(properties, tile.lod));
    return { tile.easting, tile.northing
            , tile.easting + ts, tile.northing + ts };
}

inline TileId fromAlignment(const Properties &properties, const TileId &tileId)
{
    return { tileId.lod, tileId.easting - properties.alignment(0)
            , tileId.northing - properties.alignment(1) };
}

inline TileId parent(const Alignment &alignment, long baseTileSize
                     , const TileId &tileId)
{
    auto ts(tileSize(baseTileSize, tileId.lod));
    Point2l tiled((tileId.easting - alignment(0)) / ts
                  , (tileId.northing - alignment(1)) / ts);

    constexpr auto mask(~(static_cast<decltype(tileId.easting)>(1)));

    return {
        Lod(tileId.lod - 1)
        , alignment(0) + (tiled(0) & mask) * ts
        , alignment(1) + (tiled(1) & mask) * ts
    };
}

inline TileIdChildren children(long baseTileSize, const TileId &tileId)
{
    Lod lod(tileId.lod + 1);
    auto ts(tileSize(baseTileSize, lod));

    return {{
        { lod, tileId.easting, tileId.northing }               // lower-left
        , { lod, tileId.easting + ts, tileId.northing }        // lower-right
        , { lod, tileId.easting, tileId.northing + ts }        // upper-left
        , { lod, tileId.easting + ts, tileId.northing +  ts }  // upper-right
    }};
}

inline bool operator==(const TileId &lhs, const TileId &rhs)
{
    return ((lhs.lod == rhs.lod)
            && (lhs.easting == rhs.easting)
            && (lhs.northing == rhs.northing));
}

inline bool operator!=(const TileId &lhs, const TileId &rhs)
{
    return !(lhs == rhs);
}

inline Lod deltaDown(const LodLevels &levels, Lod lod)
{
    Lod res(lod + 1);
    while (std::abs(res - levels.lod) % levels.delta) {
        ++res;
    }
    return res;
}

} } // namespace vadstena::tilestorage

#endif // vadstena_libs_tilestorage_hpp_included_
