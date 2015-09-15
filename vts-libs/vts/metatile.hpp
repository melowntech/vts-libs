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

            , allChildren = (ulChild | urChild | llChild | lrChild)

            , real = (geometryPresent | navtilePresent
                      | internalTexturePresent)
        };
    };

    bool real() const { return check(Flag::real); }

    bool geometry() const { return check(Flag::geometryPresent); }
    MetaNode& geometry(bool value) {
        return set(Flag::geometryPresent, value);
    }

    bool navtile() const { return check(Flag::navtilePresent); }
    MetaNode& navtile(bool value) {
        return set(Flag::navtilePresent, value);
    }

    bool internalTexture() const {
        return check(Flag::internalTexturePresent);
    }
    MetaNode& internalTexture(bool value) {
        return set(Flag::internalTexturePresent, value);
    }

    enum class CoarsenessControl { displaySize, texelSize };

    CoarsenessControl cc() const {
        return (check(Flag::coarsenessControl)
                ? CoarsenessControl::texelSize
                : CoarsenessControl::displaySize);
    }

    MetaNode& cc(CoarsenessControl value) {
        return set(Flag::coarsenessControl
            , (value == CoarsenessControl::displaySize));
    }

    bool ulChild() const { return check(Flag::ulChild); }
    MetaNode&  ulChild(bool value) { return set(Flag::ulChild, value); }

    bool urChild() const { return check(Flag::urChild); }
    MetaNode&  urChild(bool value) { return set(Flag::urChild, value); }

    bool llChild() const { return check(Flag::llChild); }
    MetaNode&  llChild(bool value) { return set(Flag::llChild, value); }

    bool lrlChild() const { return check(Flag::lrChild); }
    MetaNode&  lrChild(bool value) { return set(Flag::lrChild, value); }

    /** Normalized extents in range 0.0-1.0.
     */
    math::Extents3 extents;

    union {
        std::uint16_t displaySize;
        float meshArea;
    };

    float textureArea;

    Range<std::int16_t> heightRange;

    storage::CreditIds credits;

    MetaNode() : textureArea(), heightRange(), flags_() {}

    std::uint8_t flags() const { return flags_; }
    void flags(std::uint8_t flags) { flags_ = flags; }

    std::uint8_t childFlags() const { return flags_ & Flag::allChildren; }
    void childFlags(std::uint8_t flags) {
        flags_ = (flags_ & ~Flag::allChildren) | (flags & Flag::allChildren);
    }

    void update(const MetaNode &other);

    /** Sets/unsets child flag based on LSB of tileId.
     */
    MetaNode& setChildFromId(const TileId &tileId, bool value = true);

    MetaNode& mergeExtents(const MetaNode &other);

private:
    bool check(std::uint8_t flag) const { return flags_ & flag; }

    MetaNode& set(std::uint8_t flag, bool value) {
        if (value) {
            flags_ |= flag;
        } else {
            flags_ &= ~flag;
        }
        return *this;
    }

    std::uint8_t flags_;
};

class MetaTile {
public:
    typedef std::uint32_t size_type;

    MetaTile(const TileId &origin, std::uint8_t binaryOrder)
        : origin_(origin), binaryOrder_(binaryOrder)
        , size_(1 << binaryOrder)
        , grid_(size_ * size_, {})
    {}

    const MetaNode* set(const TileId &tileId, const MetaNode &node);

    const MetaNode* get(const TileId &tileId, std::nothrow_t) const;
    const MetaNode& get(const TileId &tileId) const;

    void update(const TileId &tileId, const MetaNode &mn);

    void save(std::ostream &out) const;

    void load(std::istream &in
              , const boost::filesystem::path &path = "unknown");

    const TileId& origin() const { return origin_; }

    /** Runs given function for every real tile.
     */
    template <typename F> void for_each(F f) const;

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

inline const MetaNode* MetaTile::get(const TileId &tileId, std::nothrow_t)
    const
{
    if (auto i = index(tileId, std::nothrow)) {
        return &grid_[*i];
    }
    return nullptr;
}

inline const MetaNode& MetaTile::get(const TileId &tileId)
    const
{
    return grid_[index(tileId)];
}

template <typename F>
inline void MetaTile::for_each(F f) const
{
    for (auto j(valid_.ll(1)); j < valid_.ur(1); ++j) {
        for (auto i(valid_.ll(0)); i < valid_.ur(0); ++i) {
            const auto &node(grid_[j * size_ + i]);
            f(TileId(origin_.lod, origin_.x + i, origin_.y + j), node);
        }
    }
}

} } // namespace vadstena::vts

#endif // vadstena_libs_vts_metatile_hpp
