#ifndef vadstena_libs_vts_tileindex_hpp_included_
#define vadstena_libs_vts_tileindex_hpp_included_

#include <map>

#include <boost/filesystem/path.hpp>

#include "./qtree.hpp"
#include "./basetypes.hpp"
#include "./tileop.hpp"

namespace vadstena { namespace vts {

class TileIndex {
public:
    struct Flag {
        typedef std::uint32_t value_type;

        /** NB: we can use only 7 since only byte from value is serialized and
         *  0xff has special meaning.
         */
        enum : value_type {
            mesh = 0x01
            , watertight = 0x02 // tile's mesh has no holes
            , atlas = 0x04
            , navtile = 0x08
            , alien = 0x20 // alien tile share value with reference
            , reference = 0x20
            , multimesh = 0x40 // tile's mesh has multiple submeshes

            // tile is real if it contains mesh
            , real = mesh

            // content: tile has some content
            , content = (mesh | atlas | navtile)

            // flags not present in metatiles
            , nonmeta = (watertight | multimesh)

            // cannot be 0xff since it is reserved value!
            , any = 0x7f

            // no flag set
            , none = 0x00
        };

        static bool isReal(value_type flags) { return (flags & real); };

        static bool isWatertight(value_type flags) {
            return (flags & watertight);
        };

        static bool check(value_type value, value_type mask, value_type match)
        {
            return ((value & mask) == match);
        }

        static bool check(value_type value, value_type mask)
        {
            return (value & mask);
        }

        static bool isAlien(value_type flags) {
            return check(flags, mesh | alien, mesh | alien);
        };
    };

    TileIndex() : minLod_() {}

    TileIndex(LodRange lodRange, const TileIndex *other = nullptr
              , bool noFill = false);

    template <typename Filter>
    TileIndex(LodRange lodRange, const TileIndex &other
              , const Filter &filter, bool fullRange = true);

    TileIndex(const TileIndex &other);

    struct ShallowCopy {};
    TileIndex(const TileIndex &other, ShallowCopy);

    typedef std::vector<QTree> Trees;

    void load(std::istream &is, const boost::filesystem::path &path
              = "unknown");
    void load(const boost::filesystem::path &path);

    void save(std::ostream &os) const;
    void save(const boost::filesystem::path &path) const;

    bool exists(const TileId &tileId) const { return get(tileId); }

    /** Returns true if there is any non-zero record in subtree rooted by
     *  tileId.
     */
    bool validSubtree(const TileId &tileId) const;

    /** Alternative version of validSubtree. Starts at given lod.
     */
    bool validSubtree(Lod lod, const TileId &tileId) const;

    bool real(const TileId &tileId) const {
        return (get(tileId) & Flag::real);
    }

    /** Returns whether contains real tile with given alien flag
     */
    bool real(const TileId &tileId, bool alien) const {
        auto flag(get(tileId));
        if (!(flag & Flag::real)) { return false; }
        return bool(flag & Flag::alien) == alien;
    }

    bool navtile(const TileId &tileId) const {
        return (get(tileId) & Flag::navtile);
    }

    void fill(Lod lod, const TileIndex &other);

    void fill(const TileIndex &other);

    template <typename Op>
    void fill(Lod lod, const TileIndex &other, const Op &op);

    template <typename Op>
    void fill(const TileIndex &other, const Op &op);

    void set(const TileId &tileId, QTree::value_type value);

    /** Set whole lod to given value.
     */
    void set(Lod lod, QTree::value_type value);

    /** Set all tiles in given range
     */
    void set(Lod lod, const TileRange &range, QTree::value_type value);

    /** Set all tiles in given tile range at lodRange.min and all tiles under
     *  them up to lodRange.max
     */
    void set(const LodRange &lodRange, const TileRange &range
             , QTree::value_type value);

    QTree::value_type get(const TileId &tileId) const;

    QTree::value_type checkMask(const TileId &tileId, QTree::value_type mask)
        const
    {
        return get(tileId) & mask;
    }

    QTree::value_type checkMask(const TileId &tileId, QTree::value_type allow
                                , QTree::value_type deny)
        const
    {
        auto value(get(tileId));
        return ((value & allow) && (value & ~deny));
    }

    void unset(const TileId &tileId) { set(tileId, 0); }

    /** Updates value. Equivalent to set(tileId, op(get(tileId)))
     */
    template <typename Op>
    void update(const TileId &tileId, Op op) {
        set(tileId, op(get(tileId)));
    }

    /** Updates values in given tile range.
     *
     * \param lod lod to work with
     * \param range tile range to work with
     * \param op all nodes are set to result of op(oldValue)

     * \param adding tells that result will probably set some nonzero value;
     *               used as a hint for implementation to not generate tree for
     *               given lod if we are only removing some values
     */
    template <typename Op>
    void update(Lod lod, const TileRange &range, Op op
                , bool adding = true);

    TileId tileId(Lod lod, long x, long y) const;

    math::Size2 rasterSize(Lod lod) const {
        return math::Size2(1 << lod, 1 << lod);
    }

    bool empty() const;

    Lod minLod() const { return minLod_; }

    Lod maxLod() const;

    LodRange lodRange() const;

    const Trees& trees() const { return trees_; };

    /** Clears lod content.
     */
    void clear(Lod lod);

    /** Returns count of tiles in the index.
     */
    std::size_t count() const;

    const QTree* tree(Lod lod) const;

    /** Sets value as a bit mask.
     *  Affects only bits set in mask.
     */
    void setMask(const TileId &tileId, QTree::value_type mask
                 , QTree::value_type value = Flag::any);

    struct Stat {
        storage::LodRange lodRange;
        std::vector<TileRange> tileRanges;
        std::size_t count;

        Stat()
            : lodRange(LodRange::emptyRange()), count()
        {}

        /** Computes tile range (extents) at topmost LOD taking lower LODs into
         *  account.
         */
        TileRange computeTileRange() const;
    };

    /** Get statistics for all tiles with given mask.
     */
    Stat statMask(QTree::value_type mask) const;

    /** Get statistics for all tiles with given mask and value.
     */
    Stat statMask(QTree::value_type mask, QTree::value_type value) const;

    /** Compute tile and lod range for given for all tiles satisfying given
     *  mask. Uses statMask(mask) internally.
     */
    std::pair<LodRange, TileRange> ranges(QTree::value_type mask) const;

    /** Translates node values.
     */
    template <typename Op>
    void translate(const Op &op) {
        for (auto &tree : trees_) {
            tree.translateEachNode(op);
        }
    }

    /** Creates new tile index that has given lod range and every tile from
     *  input where Filter return true has all parents up to lodRange.min and
     *  all children down to lodRange.max
     */
    TileIndex grow(const LodRange &lodRange
                   , Flag::value_type type = Flag::any) const;

    /** Intersect this index with the other.
     */
    TileIndex intersect(const TileIndex &other
                        , Flag::value_type type = Flag::any) const;

    /** Checks for any non-overlapping lod.
     */
    bool notoverlaps(const TileIndex &other
                     , Flag::value_type type = Flag::any) const;

    /** Grows this tileindex down (inplace).
     */
    TileIndex& growDown(Flag::value_type type = Flag::any);

    /** Grows this tileindex up (inplace).
     */
    TileIndex& growUp(Flag::value_type type = Flag::any);

    /** Inverts value of all tiles. If tile's mask matches then it is set to
     *  zero; if mask doesn't match it is set to type.
     */
    TileIndex& invert(Flag::value_type type = Flag::any);

    /** Converts this tileindex to simplified index (black -> 0, white -> 1)
     */
    TileIndex& simplify(Flag::value_type type = Flag::any);

    /** Makes tileindex complete -> every existing tile has its parent.
     */
    TileIndex& complete(Flag::value_type type = Flag::any);

    /** Makes tileindex complete down the tree -> every existing tile has its
     *  child.
     */
    TileIndex& completeDown(Flag::value_type type = Flag::any);

    /** Ensures that quad-tree comdition is met: every tile has its sibling.
     */
    TileIndex& round(Flag::value_type type = Flag::any);

    /** Unsets every occurence of given flag(s).
     */
    TileIndex& unset(Flag::value_type type = Flag::none);

    /** Combines this tile index with other.
     *
     * Combiner prototype:
     *
     *     Flag::value_type combiner(Flag::value_type o, Flag::value_type n)
     *
     *     \param o value currently stored in this tile index
     *     \param n value read from other tile index
     *     \return value to store
     *
     * \param other tile index to combine with
     * \param combiner combining function
     * \param lodRange limit to lod range
     */
    template <typename Combiner>
    TileIndex& combine(const TileIndex &other, const Combiner &combiner
                       , const LodRange &lodRange = LodRange::emptyRange());

    /** Trims trim levels from each lod and makes the tree complete.
     */
    TileIndex& shrinkAndComplete(unsigned int trim);

    /** Makes given lod range available.
     */
    TileIndex& makeAvailable(const LodRange &lodRange);

    /** Makes tile index an absolute tree, i.e. starting from LOD 0;
     *  equivalent to:
     *
     *      makeAvailable({ 0, maxLod });
     */
    TileIndex& makeAbsolute();

    Flag::value_type allSetFlags() const { return allSetFlags_; }

private:
    QTree* tree(Lod lod, bool create = false);

    Lod minLod_;
    Trees trees_;
    Flag::value_type allSetFlags_;
};

typedef std::vector<const TileIndex*> TileIndices;

TileIndex unite(const TileIndices &tis
                , TileIndex::Flag::value_type type = TileIndex::Flag::any
                , const LodRange &lodRange = LodRange::emptyRange());

TileIndex unite(const TileIndex &l, const TileIndex &r
                , TileIndex::Flag::value_type type = TileIndex::Flag::any
                , const LodRange &lodRange = LodRange::emptyRange());

/** Dump tile indes as set of images.
 */
void dumpAsImages(const boost::filesystem::path &path, const TileIndex &ti
                  , TileIndex::Flag::value_type type = TileIndex::Flag::any
                  , const long maxArea = 1 << 26);

// inline stuff

inline bool TileIndex::empty() const
{
    return trees_.empty();
}

inline Lod TileIndex::maxLod() const
{
    return minLod_ + trees_.size() - 1;
}

inline LodRange TileIndex::lodRange() const
{
    if (empty()) { return LodRange::emptyRange(); }
    return { minLod_, maxLod() };
}

inline const QTree* TileIndex::tree(Lod lod) const
{
    auto idx(lod - minLod_);
    if ((idx < 0) || (idx >= int(trees_.size()))) {
        return nullptr;
    }

    // get tree
    return &trees_[idx];
}

inline QTree::value_type TileIndex::get(const TileId &tileId) const
{
    const auto *m(tree(tileId.lod));
    if (!m) { return 0; }
    return m->get(tileId.x, tileId.y);
}

inline void TileIndex::set(const TileId &tileId, QTree::value_type value)
{
    if (auto *m = tree(tileId.lod, value)) {
        m->set(tileId.x, tileId.y, value);
    }
}

inline void TileIndex::set(Lod lod, QTree::value_type value)
{
    if (auto *m = tree(lod, value)) {
        m->reset(value);
    }
}

inline void TileIndex::set(Lod lod, const TileRange &range
                           , QTree::value_type value)
{
    if (auto *m = tree(lod, value)) {
        m->set(range.ll(0), range.ll(1), range.ur(0), range.ur(1), value);
    }
}

template <typename Op>
inline void traverse(const QTree &tree, Lod lod, const Op &op)
{
    tree.forEach([&](long x, long y, QTree::value_type mask)
    {
        op(TileId(lod, x, y), mask);
    }, QTree::Filter::white);
}

template <typename Op>
inline void traverse(const TileIndex &ti, const Op &op)
{
    auto lod(ti.minLod());
    for (const auto &tree : ti.trees()) {
        traverse(tree, lod++, op);
    }
}

template <typename Op>
inline void traverse(const TileIndex &ti, Lod lod, const Op &op)
{
    const auto *tree(ti.tree(lod));
    if (!tree) { return; }
    traverse(*tree, lod, op);
}

template <typename Op>
void TileIndex::fill(Lod lod, const TileIndex &other
                     , const Op &op)
{
    // find old and new trees
    const auto *oldTree(other.tree(lod));
    if (!oldTree) { return; }

    auto *newTree(tree(lod));
    if (!newTree) { return; }

    newTree->merge(*oldTree, op);
}

template <typename Op>
void TileIndex::fill(const TileIndex &other, const Op &op)
{
    for (auto lod : lodRange()) {
        fill(lod, other, op);
    }
}

template <typename Filter>
inline TileIndex::TileIndex(LodRange lodRange, const TileIndex &other
                            , const Filter &filter, bool fullRange)
{
    if (fullRange) {
        // something present in on-disk data
        lodRange = unite(lodRange, other.lodRange());
    }

    // set minimum LOD
    minLod_ = lodRange.min;

    // fill in trees
    for (auto lod : lodRange) {
        trees_.emplace_back(lod);

        fill(lod, other, filter);
    }
}

template <typename Combiner>
TileIndex& TileIndex::combine(const TileIndex &other
                              , const Combiner &combiner
                              , const LodRange &lodRange)
{
    // use provided range unless empty
    auto lr(lodRange.empty() ? other.lodRange() : lodRange);

    // process all lods from the input
    auto lod(other.minLod());
    for (const auto &input : other.trees()) {
        if (in(lod, lr)) {
            // tree for this LOD is created if non-existent
            tree(lod, true)->combine(input, combiner);
        }
        ++lod;
    }

    // done
    return *this;
}

template <typename Op>
void TileIndex::update(Lod lod, const TileRange &range, Op op
                       , bool adding)
{
    if (auto *m = tree(lod, adding)) {
        m->update(range.ll(0), range.ll(1), range.ur(0), range.ur(1), op);
    }
}

} } // namespace vadstena::vts

#endif // vadstena_libs_vts_tileindex_hpp_included_
