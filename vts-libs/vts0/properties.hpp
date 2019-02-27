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
/**
 * \file vts/properties.hpp
 * \author Vaclav Blazek <vaclav.blazek@citationtech.net>
 */

#ifndef vtslibs_vts0_properties_hpp_included_
#define vtslibs_vts0_properties_hpp_included_

#include <map>

#include <boost/optional.hpp>
#include <boost/any.hpp>

#include "basetypes.hpp"

namespace vtslibs { namespace vts0 {


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

    /** Frame of reference. Only marked here for conversion.
     */
    std::string referenceFrame;

    /** Spatial reference system of map.
     */
    std::string srs;

    /** Extents of LOD=0.
     */
    math::Extents2 extents;

    /** Driver properties.
     */
    DriverProperties driver;

    /** Horizontal scale applied in vertical direction?
     */
    bool verticalAdjustment;

    /** Mask bitfields.
     */
    struct Mask { enum {             // mask bitfields
        id = 0x001
        , metaLevels = 0x002
        , extents = 0x004
        , srs = 0x008
        , driver = 0x010
        , verticalAdjustment = 0x020
        , referenceFrame = 0x040
        // IF YOU WANT TO NEW ITEM DO NOT COLIDE WITH SettableProperties
        // AND UPDATE ALL:
        , all = (id | metaLevels | extents | srs | driver | verticalAdjustment
                 | referenceFrame)
    }; };

    typedef int MaskType;

    StaticProperties() {}

    template <typename> struct Setter;
    struct Wrapper;

    bool merge(const StaticProperties &other, MaskType mask = Mask::all);
    bool merge(const StaticProperties::Wrapper &other);
};

/** Tile set properties that can be set anytime.
 */
struct SettableProperties {
    math::Point3 defaultPosition;    // x, y, altitude
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

struct StaticProperties::Wrapper {
    StaticProperties props;
    MaskType mask;

    Wrapper() : props(), mask() {}
    Wrapper(const StaticProperties &props, MaskType mask)
        : props(props), mask(mask)
    {}

    operator StaticProperties&() { return props; }
    operator const StaticProperties&() const { return props; }
};

struct SettableProperties::Wrapper {
    SettableProperties props;
    MaskType mask;

    Wrapper() : props(), mask() {}
    Wrapper(const SettableProperties &props, MaskType mask)
        : props(props), mask(mask)
    {}

    operator SettableProperties&() { return props; }
    operator const SettableProperties&() const { return props; }
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

/** All tile set properties.
 */
struct Properties
    : StaticProperties
    , SettableProperties
{
    bool hasData;                 //!< true if any tile is present

    std::string meshTemplate;     //!< mesh file template
    std::string textureTemplate;  //!< texture file template
    std::string metaTemplate;     //!< meta tile file template

    Properties() : hasData(false) {}

    StaticProperties& staticProperties() { return *this; }
    const StaticProperties& staticProperties() const { return *this; }

    SettableProperties& settableProperties() { return *this; }
    const SettableProperties& settableProperties() const { return *this; }
};

struct CreateProperties {
public:
    typedef int MaskType;

    CreateProperties() {}

    CreateProperties(const Properties &p
                     , MaskType mask = (StaticProperties::Mask::all
                                        | SettableProperties::Mask::all))
        : staticProperties(p, mask), settableProperties(p, mask)
    {}

    StaticProperties::Wrapper staticProperties;
    SettableProperties::Wrapper settableProperties;

    StaticProperties::Setter<CreateProperties> staticSetter();
    SettableProperties::Setter<CreateProperties> settableSetter();
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
    TILESTORAGE_PROPERTIES_SETTER(extents)
    TILESTORAGE_PROPERTIES_SETTER(srs)
    TILESTORAGE_PROPERTIES_SETTER(driver)
    TILESTORAGE_PROPERTIES_SETTER(verticalAdjustment)
    TILESTORAGE_PROPERTIES_SETTER(referenceFrame)
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
    TILESTORAGE_PROPERTIES_MERGE(extents);
    TILESTORAGE_PROPERTIES_MERGE(srs);
    TILESTORAGE_PROPERTIES_MERGE(driver);
    TILESTORAGE_PROPERTIES_MERGE(verticalAdjustment)
    TILESTORAGE_PROPERTIES_MERGE(referenceFrame)

    return changed;
}

#undef TILESTORAGE_PROPERTIES_MERGE

inline bool StaticProperties::merge(const StaticProperties::Wrapper &other)
{
    return merge(other.props, other.mask);
}

inline bool SettableProperties::merge(const SettableProperties::Wrapper &other)
{
    return merge(other.props, other.mask);
}

inline StaticProperties::Setter<CreateProperties>
CreateProperties::staticSetter()
{
    return { staticProperties, this };
}

inline SettableProperties::Setter<CreateProperties>
CreateProperties::settableSetter()
{
    return { settableProperties, this };
}

} } // namespace vtslibs::vts0

#endif // vtslibs_vts0_properties_hpp_included_
