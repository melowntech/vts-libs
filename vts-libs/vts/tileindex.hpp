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
    TileIndex() : minLod_() {}

    TileIndex(LodRange lodRange, const TileIndex *other = nullptr
              , bool noFill = false);

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

private:
    QTree* tree(Lod lod, bool create = false);

    Lod minLod_;
    Trees trees_;
};

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

} } // namespace vadstena::vts

#endif // vadstena_libs_vts_tileindex_hpp_included_
