/**
 * \file vts/tileset/properties.hpp
 * \author Vaclav Blazek <vaclav.blazek@citationtech.net>
 */

#ifndef vadstena_libs_vts_tileset_properties_hpp_included_
#define vadstena_libs_vts_tileset_properties_hpp_included_

#include "../../storage/credits.hpp"
#include "../../registry.hpp"

#include "../basetypes.hpp"

namespace vadstena { namespace vts {

/** Tile set properties that must be specified during creation. They cannot be
 *  changed later.
 */
struct StaticProperties {
    /** Unique set identifier.
     */
    std::string id;

    /** Tileset's reference frame
     */
    std::string referenceFrame;

    /** Data version/revision. Should be increment anytime the data change.
     *  Used in template URL's to push through caches.
     */
    unsigned int revision;

    /** Set of credits.
     */
    registry::Credits credits;

    /** Position.
     */
    registry::Position position;

    /** Mask bitfields.
     */
    struct Mask { enum {             // mask bitfields
        id = 0x001
        , referenceFrame = 0x002
        , revision = 0x004
        , credits = 0x008
        , position = 0x010

        , all = (id | referenceFrame | revision | credits | position)
    }; };

    typedef int MaskType;

    StaticProperties() : revision(0) {}

    template <typename> struct Setter;
    struct Wrapper;

    bool merge(const StaticProperties &other, MaskType mask = Mask::all);
    bool merge(const StaticProperties::Wrapper &other);
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
    TILESTORAGE_PROPERTIES_SETTER(referenceFrame)
    TILESTORAGE_PROPERTIES_SETTER(revision)
    TILESTORAGE_PROPERTIES_SETTER(credits)
    TILESTORAGE_PROPERTIES_SETTER(position)
TILESTORAGE_PROPERTIES_SETTER_FINI()

#undef TILESTORAGE_PROPERTIES_SETTER_INIT
#undef TILESTORAGE_PROPERTIES_SETTER_FINI
#undef TILESTORAGE_PROPERTIES_SETTER

// inline stuff

#define TILESTORAGE_PROPERTIES_MERGE(WHAT)                          \
    if (mask & Mask::WHAT) { WHAT = other.WHAT; changed = true; }

inline bool StaticProperties::merge(const StaticProperties &other
                                    , MaskType mask)
{
    bool changed(false);

    TILESTORAGE_PROPERTIES_MERGE(id);
    TILESTORAGE_PROPERTIES_MERGE(referenceFrame)
    TILESTORAGE_PROPERTIES_MERGE(revision)
    TILESTORAGE_PROPERTIES_MERGE(credits)
    TILESTORAGE_PROPERTIES_MERGE(position)

    return changed;
}

#undef TILESTORAGE_PROPERTIES_MERGE

inline bool StaticProperties::merge(const StaticProperties::Wrapper &other)
{
    return merge(other.props, other.mask);
}

} } // namespace vadstena::vts

#endif // vadstena_libs_vts_tileset_properties_hpp_included_
