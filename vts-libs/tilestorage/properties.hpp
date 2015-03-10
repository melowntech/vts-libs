/**
 * \file tilestorage/properties.hpp
 * \author Vaclav Blazek <vaclav.blazek@citationtech.net>
 */

#ifndef vadstena_libs_tilestorage_properties_hpp_included_
#define vadstena_libs_tilestorage_properties_hpp_included_

#include <map>

#include <boost/optional.hpp>
#include <boost/any.hpp>

#include "./basetypes.hpp"

namespace vadstena { namespace tilestorage {


/** Driver properties.
 */
struct DriverProperties {
    /** Tile set type/driver name
     */
    std::string type;

    typedef std::map<std::string, boost::any> Options;

    /** Driver options, maps name to boost::any instance. Interpretation is
     *  driver dependent.
     */
    Options options;
};

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

    /** Driver properties.
     */
    DriverProperties driver;

    /** Mask bitfields.
     */
    struct Mask { enum {             // mask bitfields
        id = 0x001
        , metaLevels = 0x002
        , baseTileSize = 0x004
        , alignment = 0x008
        , srs = 0x010
        , driver = 0x020
        // IF YOU WANT TO NEW ITEM DO NOT COLIDE WITH SettableProperties
        // AND UPDATE ALL:
        , all = (id | metaLevels | baseTileSize | alignment | srs | driver)
    }; };

    typedef int MaskType;

    StaticProperties() : baseTileSize() {}

    bool merge(const StaticProperties &other, MaskType mask = Mask::all);

    struct Setter;
};

/** Tile set properties that can be set anytime.
 */
struct SettableProperties {
    math::Point3 defaultPosition;    // easting, northing, altitude
    math::Point3 defaultOrientation; // yaw, pitch, roll
    short textureQuality;            // JPEG quality
    float texelSize;                 // texelSize

    struct Mask { enum {             // mask bitfields
        defaultPosition = 0x040
        , defaultOrientation = 0x080
        , textureQuality = 0x100
        , texelSize = 0x200
        // IF YOU WANT TO NEW ITEM DO NOT COLIDE WITH SettableProperties
        // AND UPDATE ALL:
        , all = (defaultPosition | defaultOrientation | textureQuality
                 | texelSize)
    }; };

    typedef int MaskType;

    SettableProperties()
        : defaultOrientation(0, -90, 0)
        , textureQuality(85)
        , texelSize(0.1){}

    bool merge(const SettableProperties &other
               , MaskType mask = Mask::all);

    static MaskType all() { return ~(MaskType(0)); }

    struct Setter;
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
    typedef int MaskType;

    CreateProperties() : mask(StaticProperties::Mask::all) {}

    CreateProperties(const StaticProperties &cp)
        : staticProperties(cp), mask(StaticProperties::Mask::all)
    {}

    CreateProperties(const StaticProperties &cp
                     , const SettableProperties &sp
                     , MaskType mask = (StaticProperties::Mask::all
                                        | SettableProperties::Mask::all))
        : staticProperties(cp), settableProperties(sp), mask(mask)
    {}

    CreateProperties(const Properties &p
                     , MaskType mask = (StaticProperties::Mask::all
                                        | SettableProperties::Mask::all))
        : staticProperties(p), settableProperties(p), mask(mask)
    {}

    StaticProperties staticProperties;
    SettableProperties settableProperties;
    MaskType mask;
};

#define TILESTORAGE_PROPERTIES_SETTER(NAME)                     \
        Setter& NAME(const decltype(Properties::NAME) &value) { \
            p_.NAME = value;                                    \
            m_ |= Properties::Mask::NAME;                       \
            return *this;                                       \
        }

class StaticProperties::Setter {
public:
    typedef StaticProperties Properties;
    Setter(Properties &p, Properties::MaskType &m)
        : p_(p), m_(m)
    {}

    TILESTORAGE_PROPERTIES_SETTER(id)
    TILESTORAGE_PROPERTIES_SETTER(metaLevels)
    TILESTORAGE_PROPERTIES_SETTER(baseTileSize)
    TILESTORAGE_PROPERTIES_SETTER(alignment)
    TILESTORAGE_PROPERTIES_SETTER(srs)
    TILESTORAGE_PROPERTIES_SETTER(driver)

private:
    Properties &p_;
    Properties::MaskType &m_;
};

class SettableProperties::Setter {
public:
    typedef SettableProperties Properties;
    Setter(Properties &p, Properties::MaskType &m)
        : p_(p), m_(m)
    {}

    TILESTORAGE_PROPERTIES_SETTER(defaultPosition)
    TILESTORAGE_PROPERTIES_SETTER(defaultOrientation)
    TILESTORAGE_PROPERTIES_SETTER(textureQuality)
    TILESTORAGE_PROPERTIES_SETTER(texelSize)

private:
    Properties &p_;
    Properties::MaskType &m_;
};

#undef TILESTORAGE_PROPERTIES_SETTER

// inline stuff

#define TILESTORAGE_PROPERTIES_MERGE(WHAT)                          \
    if (mask & Mask::WHAT) { WHAT = other.WHAT; changed = true; }

inline bool SettableProperties::merge(const SettableProperties &other
                                      , MaskType mask)
{
    bool changed(false);

    TILESTORAGE_PROPERTIES_MERGE(defaultPosition);
    TILESTORAGE_PROPERTIES_MERGE(defaultOrientation);
    TILESTORAGE_PROPERTIES_MERGE(textureQuality);
    TILESTORAGE_PROPERTIES_MERGE(texelSize);

    return changed;
}

inline bool StaticProperties::merge(const StaticProperties &other
                                    , MaskType mask)
{
    bool changed(false);

    TILESTORAGE_PROPERTIES_MERGE(id);
    TILESTORAGE_PROPERTIES_MERGE(metaLevels);
    TILESTORAGE_PROPERTIES_MERGE(baseTileSize);
    TILESTORAGE_PROPERTIES_MERGE(alignment);
    TILESTORAGE_PROPERTIES_MERGE(srs);
    TILESTORAGE_PROPERTIES_MERGE(driver);

    return changed;
}

#undef TILESTORAGE_PROPERTIES_MERGE

} } // namespace vadstena::tilestorage

#endif // vadstena_libs_tilestorage_properties_hpp_included_
