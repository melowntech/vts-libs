/**
 * \file tilestorage/properties.hpp
 * \author Vaclav Blazek <vaclav.blazek@citationtech.net>
 */

#ifndef vadstena_libs_tilestorage_properties_hpp_included_
#define vadstena_libs_tilestorage_properties_hpp_included_

#include "./basetypes.hpp"

namespace vadstena { namespace tilestorage {

/** Tile set properties that must be specified during creation. They cannot be
 *  changed later.
 */
struct StaticProperties {
    /** Unique set identifier.
     */
    std::string id;

    /** Metatile lod levels (metaLevels.lod + n * metaLevels.delta).
     */
    LodLevels metaLevels;

    /** Tile size at LOD=0.
     */
    long baseTileSize;

    /** Tile alignment. No tile exists that contains this point inside.
     */
    Alignment alignment;

    /** Spatial reference system of map.
     */
    std::string srs;

    StaticProperties() : baseTileSize() {}
};

/** Tile set properties that can be set anytime.
 */
struct SettableProperties {
    math::Point3 defaultPosition;    // easting, northing, altitude
    math::Point3 defaultOrientation; // yaw, pitch, roll
    short textureQuality;            // JPEG quality

    /** Coarseness of the tileset - set as gsd.
     *  It's value is -1 if coarseness is not set to the tileset.
     */
    double coarseness;   
    /** Gsd of the tileset.
     *  It's value is -1 if coarseness is not set to the tileset.
     */            
    double gsd;                      

    struct Mask { enum {             // mask bitfields
        defaultPosition = 0x01
        , defaultOrientation = 0x02
        , textureQuality = 0x04
        , coarseness = 0x08
        , gsd = 0x16
    }; };

    typedef int MaskType;

    SettableProperties()
        : defaultOrientation(0, -90, 0)
        , textureQuality(85)
        , coarseness(-1)
        , gsd(-1) {}

    bool merge(const SettableProperties &other
               , MaskType mask = ~(MaskType(0)));
};

/** All tile set properties.
 */
struct Properties
    : StaticProperties
    , SettableProperties
{
    TileId foat;    //!< Identifier of Father-of-All-Tiles metatile
    long foatSize;  //!< Size of FOAT in meters.

    std::string meshTemplate;     //!< mesh file template
    std::string textureTemplate;  //!< texture file template
    std::string metaTemplate;     //!< meta tile file template

    Properties() : foatSize() {}
};

struct CreateProperties {
public:
    typedef SettableProperties::MaskType MaskType;

    CreateProperties() : mask(0) {}

    CreateProperties(const StaticProperties &cp)
        : staticProperties(cp), mask(0)
    {}

    CreateProperties(const StaticProperties &cp
                     , const SettableProperties &sp
                     , MaskType mask = ~(MaskType(0)))
        : staticProperties(cp), settableProperties(sp), mask(mask)
    {}

    CreateProperties(const Properties &p, MaskType mask = ~(MaskType(0)))
        : staticProperties(p), settableProperties(p), mask(mask)
    {}

    StaticProperties staticProperties;
    SettableProperties settableProperties;
    MaskType mask;
};

// inline stuff

inline bool SettableProperties::merge(const SettableProperties &other
                                      , MaskType mask)
{
    bool changed(false);
#define SETTABLEPROPERTIES_MERGE(WHAT) \
    if (mask & Mask::WHAT) { WHAT = other.WHAT; changed = true; }

    SETTABLEPROPERTIES_MERGE(defaultPosition);
    SETTABLEPROPERTIES_MERGE(defaultOrientation);
    SETTABLEPROPERTIES_MERGE(textureQuality);
    SETTABLEPROPERTIES_MERGE(coarseness);
    SETTABLEPROPERTIES_MERGE(gsd);

#undef SETTABLEPROPERTIES_MERGE
    return changed;
}

} } // namespace vadstena::tilestorage

#endif // vadstena_libs_tilestorage_properties_hpp_included_
