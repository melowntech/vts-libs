#ifndef vadstena_libs_vts_metatile_hpp
#define vadstena_libs_vts_metatile_hpp

#include <cstdint>
#include <iosfwd>
#include <vector>
#include <new>

#include <boost/filesystem/path.hpp>

#include "math/geometry_core.hpp"
#include "half/half.hpp"

#include "../storage/error.hpp"
#include "../storage/credits.hpp"

#include "./basetypes.hpp"

#include "../range.hpp"

typedef half_float::half hfloat;

namespace vadstena { namespace vts {

struct MetaNode {
    struct Flag {
        enum : std::uint8_t {
            geometryPresent = 0x01
            , navtilePresent = 0x02
            , internalTexturePresent = 0x04
            , coarsenessControl = 0x08

            , ulChild = 0x10
            , urChild = 0x20
            , llChild = 0x40
            , lrChild = 0x80
        };
    };

    bool geometry() const { return check(Flag::geometryPresent); }
    void geometry(bool value) { set(Flag::geometryPresent, value); }

    bool navtile() const { return check(Flag::navtilePresent); }
    void navtile(bool value) { set(Flag::navtilePresent, value); }

    bool internalTexture() const {
        return check(Flag::internalTexturePresent);
    }
    void internalTexture(bool value) {
        set(Flag::internalTexturePresent, value);
    }

    enum class CoarsenessControl { displaySize, texelSize };

    CoarsenessControl cc() const {
        return (check(Flag::coarsenessControl)
                ? CoarsenessControl::texelSize
                : CoarsenessControl::displaySize);
    }

    void cc(CoarsenessControl value) {
        set(Flag::coarsenessControl
            , (value == CoarsenessControl::displaySize));
    }

    bool ulChild() const { return check(Flag::ulChild); }
    void ulChild(bool value) { return set(Flag::ulChild, value); }

    bool urChild() const { return check(Flag::urChild); }
    void urChild(bool value) { return set(Flag::urChild, value); }

    bool llChild() const { return check(Flag::llChild); }
    void llChild(bool value) { return set(Flag::llChild, value); }

    bool lrlChild() const { return check(Flag::lrChild); }
    void lrChild(bool value) { return set(Flag::lrChild, value); }

    /** Normalized extents in range 0.0-1.0.
     */
    math::Extents3 extents;

    union {
        std::uint16_t displaySize;
        hfloat meshArea;
    };

    hfloat textureArea;

    Range<std::uint16_t> heightRange;

    storage::CreditIds credits;

    MetaNode() : textureArea(), heightRange(), flags_() {}

    std::uint8_t flags() const { return flags_; }
    void flags(std::uint8_t flags) { flags_ = flags; }

private:
    bool check(std::uint8_t flag) const { return flags_ & flag; }

    void set(std::uint8_t flag, bool value) {
        if (value) {
            flags_ |= flag;
        } else {
            flags_ &= ~flag;
        }
    }

    std::uint8_t flags_;
};

class MetaTile {
public:
    typedef std::uint32_t size_type;

    MetaTile(const TileId &origin, std::uint8_t binaryOrder)
        : origin_(origin), binaryOrder_(binaryOrder)
        , size_(1 << binaryOrder)
    {}

    void set(const TileId &tileId, const MetaNode &node);

    MetaNode* get(const TileId &tileId, std::nothrow_t);
    MetaNode& get(const TileId &tileId);

    void save(std::ostream &out) const;

    void load(std::istream &in
              , const boost::filesystem::path &path = "unknown");

private:
    size_type index(const TileId &tileId) const;

    boost::optional<size_type>
    index(const TileId &tileId, std::nothrow_t) const;

    boost::optional<math::Point2_<size_type> >
    gridIndex(const TileId &tileId, std::nothrow_t) const;

    math::Point2_<size_type> gridIndex(const TileId &tileId) const;

    size_type index(const math::Point2_<size_type> &gi) const;

    /** Calculates serialized node size.
     */
    /** Metatile ID, origin of first node in the metatile.
     */
    TileId origin_;

    std::uint8_t binaryOrder_;

    size_type size_;

    /** Contains (1 << binaryOrder_) * (1 << binaryOrder_) meta nodes
     */
    std::vector<MetaNode> grid_;

    /** Extents of valid area.
     */
    math::Extents2_<size_type> valid_;
};

void saveMetaTile(const boost::filesystem::path &path
                  , const MetaTile &meta);

MetaTile loadMetaTile(const boost::filesystem::path &path
                      , std::uint8_t binaryOrder);

MetaTile loadMetaTile(std::istream &in
                      , std::uint8_t binaryOrder
                      , const boost::filesystem::path &path = "unknown");


// inlines

inline MetaTile::size_type MetaTile::index(const math::Point2_<size_type> &gi)
    const
{
    return gi(1) * size_ + gi(0);
}

inline boost::optional<MetaTile::size_type>
MetaTile::index(const TileId &tileId, std::nothrow_t) const
{
    if (auto gi = gridIndex(tileId, std::nothrow)) {
        return index(*gi);
    }
    return boost::none;
}

inline MetaTile::size_type MetaTile::index(const TileId &tileId) const
{
    return index(gridIndex(tileId));
}

inline MetaNode* MetaTile::get(const TileId &tileId, std::nothrow_t)
{
    if (auto i = index(tileId, std::nothrow)) {
        return &grid_[*i];
    }
    return nullptr;
}

inline MetaNode& MetaTile::get(const TileId &tileId)
{
    return grid_[index(tileId)];
}

} } // namespace vadstena::vts

#endif // vadstena_libs_vts_metatile_hpp
