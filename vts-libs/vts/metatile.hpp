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
#ifndef vtslibs_vts_metatile_hpp
#define vtslibs_vts_metatile_hpp

#include <memory>
#include <cstdint>
#include <iosfwd>
#include <vector>
#include <new>
#include <limits>

#include <boost/optional.hpp>
#include <boost/filesystem/path.hpp>

#include "math/geometry_core.hpp"
#include "half/half.hpp"

#include "../storage/error.hpp"
#include "../storage/credits.hpp"
#include "../storage/range.hpp"

#include "basetypes.hpp"
#include "tileop.hpp"
#include "geomextents.hpp"

typedef half_float::half hfloat;

namespace vtslibs { namespace vts {

struct MetaNode {
    struct Flag {
        typedef std::uint16_t value_type;
        enum : value_type {
            geometryPresent = 0x01
            , navtilePresent = 0x02
            , applyTexelSize = 0x04
            , applyDisplaySize = 0x08

            , ulChild = 0x10
            , urChild = 0x20
            , llChild = 0x40
            , lrChild = 0x80

            , alien = 0x100

            , allChildren = (ulChild | urChild | llChild | lrChild)
            , nonChildren = value_type(~allChildren)
            , none = 0x00

            // tile is real if it contains geometry
            , real = geometryPresent
        };
    };

    bool real() const { return check(Flag::real); }

    bool real(bool alien) const {
        if (alien) {
            return check(Flag::real | Flag::alien
                         , Flag::real | Flag::alien);
        }
        return check(Flag::real | Flag::alien, Flag::real);
    }

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

    bool alien() const { return check(Flag::alien); }
    MetaNode& alien(bool value) {
        return set(Flag::alien, value);
    }

    void update(Flag::value_type flags) { flags_ |= flags; }

    void reset(Flag::value_type flags) { flags_ &= ~flags; }

    math::Extents3 extents;
    GeomExtents geomExtents;

    float texelSize;

    std::uint16_t displaySize;

    Range<std::int16_t> heightRange;

    typedef std::uint16_t SourceReference;

    SourceReference sourceReference;

    MetaNode()
        : texelSize(), displaySize(), heightRange(), sourceReference()
        , flags_(), internalTextureCount_()
    {}

    Flag::value_type flags() const { return flags_; }
    void flags(Flag::value_type flags) { flags_ = flags; }

    Flag::value_type childFlags() const { return flags_ & Flag::allChildren; }
    void childFlags(Flag::value_type flags) {
        flags_ = (flags_ & Flag::nonChildren) | (flags & Flag::allChildren);
    }

    Flag::value_type nonChildFlags() const {
        return flags_ & Flag::nonChildren;
    }
    void nonChildFlags(Flag::value_type flags) {
        flags_ = (flags_ & Flag::allChildren) | (flags & Flag::nonChildren);
    }

    void update(const MetaNode &other);

    /** Sets/unsets child flag based on LSB of tileId.
     */
    MetaNode& setChildFromId(const TileId &tileId, bool value = true);

    /** Sets/unsets child flag based on LSB of tileId in external flags.
     */
    static void setChildFromId(Flag::value_type &flags, const TileId &tileId
                               , bool value = true);

    /** Merge-in child flags. Combines current and new flags.
     */
    void mergeChildFlags(Flag::value_type cf);

    MetaNode& mergeExtents(const MetaNode &other);
    MetaNode& mergeExtents(const math::Extents3 &other);
    MetaNode& mergeExtents(const GeomExtents &other);

    unsigned int internalTextureCount() const { return internalTextureCount_; }
    void internalTextureCount(unsigned int value) {
        const unsigned int max(std::numeric_limits<std::uint8_t>::max());
        if (value > max) {
            internalTextureCount_ = max;
        } else {
            internalTextureCount_ = value;
        }
    }

    const storage::CreditIds& credits() const {  return credits_; }

    template <typename CreditIdsType>
    void updateCredits(const CreditIdsType &credits) {
        credits_.insert(credits.begin(), credits.end());
    }

    template <typename CreditIdType>
    void addCredit(const CreditIdType &creditId) {
        credits_.insert(creditId);
    }

    template <typename CreditIdsType>
    void setCredits(const CreditIdsType &credits) {
        credits_.clear();
        credits_.insert(credits.begin(), credits.end());
    }

    enum class BackingType { none, uint8, uint16 };

    struct StoreParams {
        Lod lod;
        BackingType sourceReference;

        StoreParams(Lod lod, BackingType sourceReference)
            : lod(lod), sourceReference(sourceReference) {}
    };

    void load(std::istream &in, const StoreParams &sp
              , std::uint16_t version);
    void save(std::ostream &out, const StoreParams &sp) const;

private:
    bool check(Flag::value_type flag) const { return flags_ & flag; }
    bool check(Flag::value_type flag, Flag::value_type value) const {
        return (flags_ & flag) == value;
    }

    MetaNode& set(Flag::value_type flag, bool value) {
        if (value) {
            flags_ |= flag;
        } else {
            flags_ &= ~flag;
        }
        return *this;
    }

    Flag::value_type flags_;

    storage::CreditIds credits_;

    std::uint8_t internalTextureCount_;
};

class MetaTile {
public:
    typedef std::vector<MetaTile> list;
    typedef std::shared_ptr<MetaTile> pointer;

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

    /** Runs given function for every real tile.
     */
    template <typename F> void for_each(F f);

    typedef std::vector<int> Indices;

    /** Updates this metatile with data from input metatiles while resolving
     *  references.
     *
     *  Existing real nodes are not touched except updating geometry extents
     *  from virtual nodes.
     *
     *  NB: child flags are not copied, it is up to the user to obtain values
     *  from another source.
     *
     *  Flag alien:
     *     false: take only non-alien nodes into account
     *     true: take only alien nodes into account
     *
     * \param in input metatile
     * \param alien marks processing of regular or alien nodes
     */
    void update(const MetaTile &in, bool alien = false);


    /** Sets expected reference in metanode at tileId.
     */
    void expectReference(const TileId &tileId
                         , MetaNode::SourceReference sourceReference);

    /** Update this metatile with another metatile honoring references.
     *
     *  NB: child flags are merged (union) and alien flag is always cleared.
     *
     * \param sourceReference update metatile's source reference
     * \param in update metatile
     */
    void update(MetaNode::SourceReference sourceReference, const MetaTile &in);

    bool empty() const { return !math::valid(valid_); }

    /** Current metatile version.
     */
    static int currentVersion();

    static int loadVersion(std::istream &in
                           , const boost::filesystem::path &path = "unknown");

protected:
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

MetaTile loadMetaTile(std::istream &in, std::uint8_t binaryOrder
                      , const boost::filesystem::path &path = "unknown");

MetaTile::pointer loadMetaTile(const boost::filesystem::path *path
                               , std::uint8_t binaryOrder);

MetaTile::pointer loadMetaTile(std::istream *in
                               , std::uint8_t binaryOrder
                               , const boost::filesystem::path
                               &path = "unknown");

std::vector<TileId> children(const MetaNode &node
                             , const TileId &tileId);

void loadCreditsFromMetaTile(std::istream &in, registry::IdSet &credits
                             , const boost::filesystem::path
                             &path = "unknown");

typedef std::pair<double, std::size_t> TexelSizeAggregator;

double average(const TexelSizeAggregator &aa);

void add(TexelSizeAggregator &aa, const MetaNode &node);

void add(TexelSizeAggregator &aa, const MetaNode &node
         , const MetaNode &oldNode);

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
inline void MetaTile::for_each(F f)
{
    for (auto j(valid_.ll(1)); j <= valid_.ur(1); ++j) {
        for (auto i(valid_.ll(0)); i <= valid_.ur(0); ++i) {
            auto &node(grid_[j * size_ + i]);
            f(TileId(origin_.lod, origin_.x + i, origin_.y + j), node);
        }
    }
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

inline double average(const TexelSizeAggregator &aa) {
    if (!aa.second) { return 0.0; }
    return aa.first / aa.second;
}

inline void add(TexelSizeAggregator &aa, const MetaNode &node) {
    if (node.applyTexelSize()) {
        aa.first += node.texelSize;
        ++aa.second;
    }
}

inline void add(TexelSizeAggregator &aa, const MetaNode &node
                , const MetaNode &oldNode)
{
    // remove old node first
    if (oldNode.applyTexelSize()) {
        aa.first -= oldNode.texelSize;
        --aa.second;
    }

    // and add this node
    add(aa, node);
}

} } // namespace vtslibs::vts

#endif // vtslibs_vts_metatile_hpp
