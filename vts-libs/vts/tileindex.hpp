#ifndef vadstena_libs_vts_tileindex_hpp_included_
#define vadstena_libs_vts_tileindex_hpp_included_

#include <map>

#include <boost/filesystem/path.hpp>

#include "./qtree.hpp"
#include "./basetypes.hpp"
#include "./tileop.hpp"

#include "../entities.hpp"

namespace vadstena { namespace vts {

class TileIndex {
public:
    struct TileFlag {
        enum : std::uint8_t {
            mesh = 0x01
            , watertight = 0x02
            , atlas = 0x04
            , navtile = 0x08
            , meta = 0x10
            , real = mesh | atlas | navtile
        };
    };

    TileIndex() : minLod_() {}

    TileIndex(LodRange lodRange, const TileIndex *other = nullptr
              , bool noFill = false);

    template <typename Filter>
    TileIndex(LodRange lodRange, const TileIndex &other
              , const Filter &filter);

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

    void fill(Lod lod, const TileIndex &other);

    void fill(const TileIndex &other);

    template <typename Op>
    void fill(Lod lod, const TileIndex &other, const Op &op);

    template <typename Op>
    void fill(const TileIndex &other, const Op &op);

    void set(const TileId &tileId, QTree::value_type value);

    QTree::value_type get(const TileId &tileId) const;

    QTree::value_type checkMask(const TileId &tileId, QTree::value_type mask)
        const
    {
        return get(tileId) & mask;
    }

    void unset(const TileId &tileId) { set(tileId, 0); }

    /** Updates value. Equivalent to set(tileId, op(get(tileId)))
     */
    template <typename Op>
    void update(const TileId &tileId, Op op) {
        set(tileId, op(get(tileId)));
    }

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

    void setMask(const TileId &tileId, QTree::value_type mask, bool on = true);

    struct Stat {
        storage::LodRange lodRange;
        std::vector<TileRange> tileRanges;

        Stat()
            : lodRange(LodRange::emptyRange())
        {}
    };

    /** Get statistics for all tiles with given mask.
     */
    Stat statMask(QTree::value_type mask) const;

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
    template <typename Filter>
    TileIndex grow(const LodRange &lodRange, const Filter &filter) const;

    /** Intersect this index with the other.
     */
    template <typename Filter>
    TileIndex intersect(const TileIndex &other, const Filter &filter) const;

    /** Checks for any non-overlapping lod.
     */
    template <typename Filter>
    bool notoverlaps(const TileIndex &other, const Filter &filter) const;

private:
    QTree* tree(Lod lod, bool create = false);

    Lod minLod_;
    Trees trees_;
};

typedef std::vector<const TileIndex*> TileIndices;

template <typename Op>
TileIndex unite(const TileIndices &tis, const Op &op
                , const LodRange &lodRange = LodRange::emptyRange());

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
    if (auto *m = tree(tileId.lod, true)) {
        m->set(tileId.x, tileId.y, value);
    }
}

template <typename Op>
inline void traverse(const TileIndex &ti, const Op &op)
{
    auto lod(ti.minLod());
    for (const auto &tree : ti.trees()) {
        tree.forEach([&](long x, long y, QTree::value_type mask)
        {
            op(TileId(lod, x, y), mask);
        }, QTree::Filter::white);
        ++lod;
    }
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

template <typename Op>
inline TileIndex unite(const TileIndices &tis, const Op &op
                       , const LodRange &lodRange)
{
    if (tis.empty()) {
        return TileIndex(lodRange);
    }

    auto lr(lodRange);
    for (const auto *ti : tis) {
        lr = unite(lr, ti->lodRange());
    }

    LOG(info1) << "unite: lodRange: " << lr;

    // result tile index
    TileIndex out(lr);

    // fill in targets
    for (const auto *ti : tis) {
        out.fill(*ti, op);
    }

    // done
    return out;
}

template <typename Filter>
inline TileIndex::TileIndex(LodRange lodRange, const TileIndex &other
                            , const Filter &filter)
{
    // include old definition if non-empty
    if (other.empty()) {
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

template <typename Filter>
inline TileIndex TileIndex::grow(const LodRange &lodRange
                                 , const Filter &filter) const
{
    TileIndex ti(lodRange, *this, filter);

    if (trees_.size() < 2) {
        // nothing to grow
        return ti;
    }

    // traverse trees top to bottom and refine -> propagates tiles from top
    // to bottom
    {
        auto lod(lodRange.min);
        auto ctrees(ti.trees_.begin());

        for (auto itrees(ctrees + 1), etrees(ti.trees_.end());
             itrees != etrees; ++itrees, ++ctrees, ++lod)
        {
            LOG(debug) << "gd: " << lod << " -> " << (lod + 1);

            auto &tree(*itrees);

            // merge in parent -> all children are set
            tree.merge(*ctrees, filter);
        }
    }

    // traverse trees bottom to top and coarsen -> propagates tiles from bottom
    // to top
    {
        auto lod(lodRange.max);
        auto ctrees(ti.trees_.rbegin());

        for (auto itrees(ctrees + 1), etrees(ti.trees_.rend());
             itrees != etrees; ++itrees, ++ctrees, --lod)
        {
            LOG(debug) << "gu: " << lod << " -> " << (lod - 1);

            // make copy of child
            QTree child(*ctrees);
            auto &tree(*itrees);

            // coarsen child (do not change child!)
            child.coarsen(filter);
            // merge in coarsened child -> all parents are set
            tree.merge(child, filter);
        }
    }

    return ti;
}

template <typename Filter>
TileIndex TileIndex::intersect(const TileIndex &other, const Filter &filter)
    const
{
    TileIndex ti(lodRange(), *this, filter);

    if (ti.empty()) {
        // nothing to intersect
        return ti;
    }

    auto ntree(ti.trees_.begin());
    for (auto lod : lodRange()) {
        if (const auto *otree = other.tree(lod)) {
            ntree->intersect(*otree, filter);
        }
        ++ntree;
    }
    return ti;
}

template <typename Filter>
bool TileIndex::notoverlaps(const TileIndex &other, const Filter &filter)
    const
{
    // different lod range -> mismatch
    if (lodRange() != other.lodRange()) { return true; }

    auto ntree(trees_.begin());
    for (const auto &otree : other.trees_) {
        if (!ntree->overlaps(otree, filter)) {
            return true;
        }
        ++ntree;
    }
    return false;
}

} } // namespace vadstena::vts

#endif // vadstena_libs_vts_tileindex_hpp_included_
