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
#include "./tileop.hpp"

#include "vts-libs/storage/range.hpp"

typedef half_float::half hfloat;

namespace vadstena { namespace vts {

struct MetaNode {
    struct Flag {
        typedef std::uint8_t value_type;
        enum : value_type {
            geometryPresent = 0x01
            , navtilePresent = 0x02
            , applyTexelSize = 0x04
            , applyDisplaySize = 0x08

            , ulChild = 0x10
            , urChild = 0x20
            , llChild = 0x40
            , lrChild = 0x80

            , allChildren = (ulChild | urChild | llChild | lrChild)

            // tile is real if it contains geometry
            , real = geometryPresent
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

    bool applyTexelSize() const { return check(Flag::applyTexelSize); }
    MetaNode& applyTexelSize(bool value) {
        return set(Flag::applyTexelSize, value);
    }

    bool applyDisplaySize() const { return check(Flag::applyDisplaySize); }
    MetaNode& applyDisplaySize(bool value) {
        return set(Flag::applyDisplaySize, value);
    }

    bool ulChild() const { return check(Flag::ulChild); }
    MetaNode& ulChild(bool value) { return set(Flag::ulChild, value); }

    bool urChild() const { return check(Flag::urChild); }
    MetaNode& urChild(bool value) { return set(Flag::urChild, value); }

    bool llChild() const { return check(Flag::llChild); }
    MetaNode& llChild(bool value) { return set(Flag::llChild, value); }

    bool lrlChild() const { return check(Flag::lrChild); }
    MetaNode& lrChild(bool value) { return set(Flag::lrChild, value); }

    /** Normalized extents in range 0.0-1.0.
     */
    math::Extents3 extents;

    float texelSize;

    std::uint16_t displaySize;

    Range<std::int16_t> heightRange;

    MetaNode()
        : texelSize(), displaySize(), heightRange(), flags_()
        , internalTextureCount_()
    {}

    Flag::value_type flags() const { return flags_; }
    void flags(Flag::value_type flags) { flags_ = flags; }

    Flag::value_type childFlags() const { return flags_ & Flag::allChildren; }
    void childFlags(Flag::value_type flags) {
        flags_ = (flags_ & ~Flag::allChildren) | (flags & Flag::allChildren);
    }

    void update(const MetaNode &other);

    /** Sets/unsets child flag based on LSB of tileId.
     */
    MetaNode& setChildFromId(const TileId &tileId, bool value = true);

    MetaNode& mergeExtents(const MetaNode &other);

    const storage::CreditIds& credits() const {  return credits_; }

    template <typename CreditIdsType>
    void updateCredits(const CreditIdsType &credits) {
        credits_.insert(credits.begin(), credits.end());
    }

    template <typename CreditIdType>
    void addCredit(const CreditIdType &creditId) {
        credits_.insert(creditId);
    }

    std::size_t internalTextureCount() const {
        return (geometry() ? internalTextureCount_ : 0);
    }
    MetaNode& internalTextureCount(std::size_t value);

    std::size_t reference() const {
        return (!geometry() ? reference_ : 0);
    }
    MetaNode& reference(std::size_t value);

    void load(std::istream &in, Lod lod);
    void save(std::ostream &out, Lod lod) const;

private:
    bool check(Flag::value_type flag) const { return flags_ & flag; }

    MetaNode& set(Flag::value_type flag, bool value) {
        if (value) {
            flags_ |= flag;
        } else {
            flags_ &= ~flag;
        }
        return *this;
    }

    Flag::value_type flags_;

    // union 1
    union {
        std::uint8_t internalTextureCount_;
        std::uint8_t reference_;
    };

    storage::CreditIds credits_;
};

class MetaTile {
public:
    typedef std::uint32_t size_type;
    typedef math::Point2_<size_type> point_type;
    typedef math::Size2_<size_type> size2_type;
    typedef math::Extents2_<size_type> extents_type;

    MetaTile(const TileId &origin, std::uint8_t binaryOrder)
        : origin_(origin), binaryOrder_(binaryOrder)
        , size_(1 << binaryOrder)
        , grid_(size_ * size_, MetaNode())
        , valid_(math::InvalidExtents{})
    {}

    const MetaNode* set(const TileId &tileId, const MetaNode &node);

    const MetaNode* get(const TileId &tileId, std::nothrow_t) const;
    const MetaNode& get(const TileId &tileId) const;

    void update(const TileId &tileId, const MetaNode &mn);

    void save(std::ostream &out) const;

    void load(std::istream &in
              , const boost::filesystem::path &path = "unknown");

    const TileId& origin() const { return origin_; }

    const size_type& size() const { return size_; }

    extents_type validExtents() const;

    /** Runs given function for every real tile.
     */
    template <typename F> void for_each(F f) const;

private:
    size_type index(const TileId &tileId, bool checkValidity = true) const;

    boost::optional<size_type>
    index(const TileId &tileId, std::nothrow_t, bool checkValidity = true)
        const;

    boost::optional<point_type>
    gridIndex(const TileId &tileId, std::nothrow_t, bool checkValidity = true)
        const;

    point_type gridIndex(const TileId &tileId, bool checkValidity = true)
        const;

    size_type index(const point_type &gi) const;

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
    extents_type valid_;
};

void saveMetaTile(const boost::filesystem::path &path
                  , const MetaTile &meta);

MetaTile loadMetaTile(const boost::filesystem::path &path
                      , std::uint8_t binaryOrder);

MetaTile loadMetaTile(std::istream &in
                      , std::uint8_t binaryOrder
                      , const boost::filesystem::path &path = "unknown");

std::vector<TileId> children(const MetaNode &node
                             , const TileId &tileId);

// inlines

inline MetaTile::size_type MetaTile::index(const math::Point2_<size_type> &gi)
    const
{
    return gi(1) * size_ + gi(0);
}

inline boost::optional<MetaTile::size_type>
MetaTile::index(const TileId &tileId, std::nothrow_t
                , bool checkValidity) const
{
    if (auto gi = gridIndex(tileId, std::nothrow, checkValidity)) {
        return index(*gi);
    }
    return boost::none;
}

inline MetaTile::size_type MetaTile::index(const TileId &tileId
                                           , bool checkValidity) const
{
    return index(gridIndex(tileId, checkValidity));
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
    for (auto j(valid_.ll(1)); j <= valid_.ur(1); ++j) {
        for (auto i(valid_.ll(0)); i <= valid_.ur(0); ++i) {
            const auto &node(grid_[j * size_ + i]);
            f(TileId(origin_.lod, origin_.x + i, origin_.y + j), node);
        }
    }
}

inline std::vector<TileId> children(const MetaNode &node
                                    , const TileId &tileId)
{
    std::vector<TileId> c;
    std::uint8_t mask(MetaNode::Flag::ulChild);
    for (const auto &child : vts::children(tileId)) {
        if (mask & node.flags()) {
            c.push_back(child);
        }
        mask <<= 1;
    }
    return c;
}

} } // namespace vadstena::vts

#endif // vadstena_libs_vts_metatile_hpp
