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

    template <typename> struct Setter;
    struct Wrapper;

    bool merge(const StaticProperties &other, MaskType mask = Mask::all);
    bool merge(const StaticProperties::Wrapper &other);
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

    static MaskType all() { return Mask::all; }

    template <typename> struct Setter;
    struct Wrapper;

    bool merge(const SettableProperties &other, MaskType mask = Mask::all);
    bool merge(const SettableProperties::Wrapper &other);
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

    StaticProperties& staticProperties() { return *this; }
    const StaticProperties& staticProperties() const { return *this; }

    SettableProperties& settableProperties() { return *this; }
    const SettableProperties& settableProperties() const { return *this; }
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

struct StaticProperties::Wrapper {
    StaticProperties props;
    MaskType mask;

    Wrapper() : props(), mask() {}
    Wrapper(const StaticProperties &props, MaskType mask)
        : props(props), mask(mask)
    {}
};

struct SettableProperties::Wrapper {
    SettableProperties props;
    MaskType mask;

    Wrapper() : props(), mask() {}
    Wrapper(const SettableProperties &props, MaskType mask)
        : props(props), mask(mask)
    {}
};

template <typename T, typename C>
class SetterBase {
public:
    typedef T Properties;
    typedef C Context;

    SetterBase(Properties &p, typename Properties::MaskType &m
               , Context *c)
        : p_(p), m_(m), c_(c)
    {}

    Context& context() { return *c_; }

protected:
    Properties &p_;
    typename Properties::MaskType &m_;
    Context *c_;
};

#define TILESTORAGE_PROPERTIES_SETTER(NAME)                     \
        Setter& NAME(const decltype(Properties::NAME) &value) { \
            this->p_.NAME = value;                              \
            this->m_ |= Properties::Mask::NAME;                 \
            return *this;                                       \
        }

#define TILESTORAGE_PROPERTIES_SETTER_INIT(PropertiesType)          \
    template <typename Context>                                     \
    class PropertiesType::Setter                                    \
            : public SetterBase<PropertiesType, Context>            \
    {                                                               \
    public:                                                         \
        typedef SetterBase<PropertiesType, Context> Super;          \
        typedef typename Super::Properties Properties;              \
        typedef typename Properties::MaskType MaskType;             \
                                                                    \
        Setter(Properties &p, MaskType &m, Context *c = nullptr)    \
            : SetterBase<Properties, Context>(p, m, c)              \
        {}                                                          \
                                                                    \
        Setter(typename Properties::Wrapper &w, Context *c = nullptr)   \
            : SetterBase<Properties, Context>(w.props, w.mask, c)       \
        {}

#define TILESTORAGE_PROPERTIES_SETTER_FINI() };

TILESTORAGE_PROPERTIES_SETTER_INIT(StaticProperties)
    TILESTORAGE_PROPERTIES_SETTER(id)
    TILESTORAGE_PROPERTIES_SETTER(metaLevels)
    TILESTORAGE_PROPERTIES_SETTER(baseTileSize)
    TILESTORAGE_PROPERTIES_SETTER(alignment)
    TILESTORAGE_PROPERTIES_SETTER(srs)
    TILESTORAGE_PROPERTIES_SETTER(driver)
TILESTORAGE_PROPERTIES_SETTER_FINI()

TILESTORAGE_PROPERTIES_SETTER_INIT(SettableProperties)
    TILESTORAGE_PROPERTIES_SETTER(defaultPosition)
    TILESTORAGE_PROPERTIES_SETTER(defaultOrientation)
    TILESTORAGE_PROPERTIES_SETTER(textureQuality)
    TILESTORAGE_PROPERTIES_SETTER(texelSize)
TILESTORAGE_PROPERTIES_SETTER_FINI()

#undef TILESTORAGE_PROPERTIES_SETTER_INIT
#undef TILESTORAGE_PROPERTIES_SETTER_FINI
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

inline bool StaticProperties::merge(const StaticProperties::Wrapper &other)
{
    return merge(other.props, other.mask);
}

inline bool SettableProperties::merge(const SettableProperties::Wrapper &other)
{
    return merge(other.props, other.mask);
}

#undef TILESTORAGE_PROPERTIES_MERGE

} } // namespace vadstena::tilestorage

#endif // vadstena_libs_tilestorage_properties_hpp_included_
