#include <boost/filesystem.hpp>
#include <boost/format.hpp>

#include "dbglog/dbglog.hpp"
#include "utility/binaryio.hpp"

#include "../storage/error.hpp"

#include "./tileindex.hpp"
#include "./tileop.hpp"
#include "./io.hpp"
#include "./tileindex-io.hpp"

namespace vadstena { namespace vts {

namespace fs = boost::filesystem;

namespace {
    const char TILE_INDEX_IO_MAGIC[2] = { 'T', 'I' };
}

TileIndex::TileIndex(const TileIndex &other)
    : minLod_(other.minLod_)
    , trees_(other.trees_)
    , allSetFlags_()
{
}

TileIndex::TileIndex(const TileIndex &other, ShallowCopy)
    : minLod_(other.minLod_)
    , allSetFlags_()
{
    trees_.reserve(other.trees_.size());
    for (const auto &tree : other.trees_) {
        trees_.emplace_back(tree.order());
    }
}

TileIndex::TileIndex(LodRange lodRange
                     , const TileIndex *other, bool noFill)
    : allSetFlags_()
{
    // include old definition if non-empty
    if (other && !other->empty()) {
        // something present in on-disk data
        lodRange = unite(lodRange, other->lodRange());
    }

    // set minimum LOD
    minLod_ = lodRange.min;

    // fill in trees
    for (auto lod : lodRange) {
        trees_.emplace_back(lod);

        // fill in old data (if exists)
        if (other && !noFill) {
            fill(lod, *other);
        };
    }
}

void TileIndex::fill(Lod lod, const TileIndex &other)
{
    // find old and new trees
    const auto *oldTree(other.tree(lod));
    if (!oldTree) { return; }

    auto *newTree(tree(lod));
    if (!newTree) { return; }

    newTree->merge(*oldTree, [](QTree::value_type value) { return value; });
}

void TileIndex::fill(const TileIndex &other)
{
    for (auto lod : lodRange()) {
        fill(lod, other);
    }
}

namespace {

bool checkMagic(std::istream &f)
{
    using utility::binaryio::read;

    char magic[sizeof(TILE_INDEX_IO_MAGIC)];
    read(f, magic);
    if (!f) { return false; }

    if (std::memcmp(magic, TILE_INDEX_IO_MAGIC, sizeof(TILE_INDEX_IO_MAGIC))) {
        return false;
    }
    return true;
}

} // namespace

void TileIndex::load(std::istream &f, const fs::path &path)
{
    using utility::binaryio::read;

    if (!checkMagic(f)) {
        LOGTHROW(err2, storage::Error)
            << "TileIndex " << path << " has wrong magic.";
    }

    uint8_t minLod, size;
    read(f, minLod);
    read(f, size);
    minLod_ = minLod;

    trees_.resize(size);
    allSetFlags_ = 0;
    for (auto &tree : trees_) {
        tree.load(f, path);
        allSetFlags_ |= tree.allSetFlags();
    }
}

void TileIndex::load(const fs::path &path)
{
    std::ifstream f;
    f.exceptions(std::ifstream::failbit | std::ifstream::badbit);

    try {
        f.open(path.string(), std::ios_base::in | std::ifstream::binary);
        f.peek();
    } catch (const std::exception &e) {
        LOGTHROW(err1, storage::Error)
            << "Unable to open tileindex " << path << ".";
    }

    load(f, path);

    f.close();
}

void TileIndex::save(std::ostream &f) const
{
    using utility::binaryio::write;

    write(f, TILE_INDEX_IO_MAGIC); // 7 bytes

    write(f, uint8_t(minLod_));
    write(f, uint8_t(trees_.size()));

    // save lod-tree mapping
    for (const auto &tree : trees_) {
        tree.save(f);
    }
}

void TileIndex::save(const fs::path &path) const
{
    utility::ofstreambuf f;
    f.exceptions(std::ifstream::failbit | std::ifstream::badbit);
    f.open(path.string(), std::ifstream::out | std::ifstream::trunc);

    save(f);

    f.close();
}

void TileIndex::clear(Lod lod)
{
    if (auto *m = tree(lod)) {
        m->recreate(m->order());
    }
}

QTree* TileIndex::tree(Lod lod, bool create)
{
    auto idx(lod - minLod_);
    if (!create) {
        if ((idx < 0) || (idx >= int(trees_.size()))) {
            return nullptr;
        }

        // get tree
        return &trees_[idx];
    }

    // first lod? just add
    if (trees_.empty()) {
        LOG(debug) << "Creating first lod: " << lod;
        minLod_ = lod;
        trees_.emplace_back(lod);

    } else if (lod < minLod_) {
        // LOD too low
        // generate all tree up to given LOD
        Trees trees;
        for (Lod l(lod); l < minLod_; ++l) {
            LOG(debug) << "Prepending lod: " << l;
            trees.emplace_back(l);
        }
        // prepend all generated trees befor existing
        trees_.insert(trees_.begin(), trees.begin(), trees.end());
        minLod_ = lod;

    } else if (lod >= (minLod_ + int(trees_.size()))) {
        // LOD too high
        // generate all tree up to given LOD
        for (Lod l(minLod_ + trees_.size()); l <= lod; ++l) {
            LOG(debug) << "Appending lod: " << l;
            trees_.emplace_back(l);
        }
    }

    // get tree
    return &trees_[lod - minLod_];
}

std::size_t TileIndex::count() const
{
    std::size_t total(0);
    for (const auto &tree : trees_) {
        total += tree.count();
    }
    return total;
}

void TileIndex::setMask(const TileId &tileId, QTree::value_type mask
                        , QTree::value_type value)
{
    update(tileId, [&](QTree::value_type old) -> QTree::value_type
    {
        return (old & ~mask) | (value & mask);
    });
}

TileIndex::Stat TileIndex::statMask(QTree::value_type mask)
    const
{
    Stat stat;

    auto lod(minLod_);
    for (const auto &tree : trees_) {
        stat.tileRanges.emplace_back(math::InvalidExtents{});
        auto &tileRange(stat.tileRanges.back());

        tree.forEachNode([&](unsigned int x, unsigned int y, unsigned int size
                             , QTree::value_type v)
        {
            if (!(v & mask)) { return; }

            // remember lod
            storage::update(stat.lodRange, lod);

            // update tile range
            math::update(tileRange, x, y);
            math::update(tileRange, x + size - 1, y + size - 1);

            stat.count += (std::size_t(size) * std::size_t(size));
        });
        ++lod;
    }

    if (stat.lodRange.empty()) {
        // nothing, drop whole tileRanges
        stat.tileRanges.clear();
    } else {
        // remove difference between minLod_ and stat.lodRange.min (if any)
        stat.tileRanges.erase
            (stat.tileRanges.begin()
             , stat.tileRanges.begin() + (stat.lodRange.min - minLod_));
    }

    return stat;
}

TileIndex::Stat TileIndex::statMask(QTree::value_type mask
                                    , QTree::value_type value)
    const
{
    Stat stat;

    auto lod(minLod_);
    for (const auto &tree : trees_) {
        stat.tileRanges.emplace_back(math::InvalidExtents{});
        auto &tileRange(stat.tileRanges.back());

        tree.forEachNode([&](unsigned int x, unsigned int y, unsigned int size
                             , QTree::value_type v)
        {
            if ((v & mask) != value) { return; }

            // remember lod
            storage::update(stat.lodRange, lod);

            // update tile range
            math::update(tileRange, x, y);
            math::update(tileRange, x + size - 1, y + size - 1);

            stat.count += (std::size_t(size) * std::size_t(size));
        });
        ++lod;
    }

    if (stat.lodRange.empty()) {
        // nothing, drop whole tileRanges
        stat.tileRanges.clear();
    } else {
        // remove difference between minLod_ and stat.lodRange.min (if any)
        stat.tileRanges.erase
            (stat.tileRanges.begin()
             , stat.tileRanges.begin() + (stat.lodRange.min - minLod_));
    }

    return stat;
}

TileIndex TileIndex::grow(const LodRange &lodRange
                          , Flag::value_type type) const
{
    auto filter([type](QTree::value_type value) { return (value & type); });
    TileIndex ti(lodRange, *this, filter, false);

    // propagate tiles down
    ti.completeDown();
    // propagate tiles up
    ti.complete();

    return ti;
}

TileIndex TileIndex::intersect(const TileIndex &other
                               , Flag::value_type type)
    const
{
    auto filter([type](QTree::value_type value) { return (value & type); });
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

bool TileIndex::notoverlaps(const TileIndex &other, Flag::value_type type)
    const
{
    auto filter([type](QTree::value_type value) { return (value & type); });

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

TileIndex unite(const TileIndices &tis, TileIndex::Flag::value_type type
                , const LodRange &lodRange)
{
    auto filter([type](QTree::value_type value) { return (value & type); });

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
        out.fill(*ti, filter);
    }

    // done
    return out;
}

TileIndex unite(const TileIndex &l, const TileIndex &r
                , TileIndex::Flag::value_type type
                , const LodRange &lodRange)
{
    return unite({&l, &r}, type, lodRange);
}

namespace {

void showTree(const QTree &tree, const std::string &name
              , QTree::value_type mask)
{
    tree.forEachNode([&](unsigned int x, unsigned int y, unsigned int size
                         , QTree::value_type value)
    {
        if (!(value & mask)) { return; }
        LOG(info4)
            << name << "[" << tree.size()
            << "] node(" << x << ", " << y << ", " << size << ")";
    });
}

} // namespace

TileIndex& TileIndex::growUp(Flag::value_type type)
{
    auto filter([type](QTree::value_type value) { return (value & type); });

    if (trees_.empty()) { return *this; }

    // traverse trees bottom to top
    auto lod(lodRange().max);

    auto itrees(trees_.rbegin());
    for (auto ptrees(itrees + 1), etrees(trees_.rend());
         itrees != etrees; ++itrees, --lod)
    {
        LOG(debug) << "gu: " << lod << " -> " << (lod - 1);

        // coarsen this tree (kill all 1-pixel nodes) -> each tile has its
        // sibling
        itrees->coarsen(filter);

        if (ptrees != etrees) {
            // if there is a parent tree: merge this coarsened tree into parent
            // tree -> each tile gets parent
            ptrees->merge(*itrees, filter);

            // increment here, otherwise ptrees would point after end
            ++ptrees;
        }
    }

    return *this;
}

TileIndex& TileIndex::completeDown(Flag::value_type type)
{
    auto filter([type](QTree::value_type value) { return (value & type); });

    if (trees_.empty()) { return *this; }

    // traverse trees top to bottom
    auto lod(lodRange().min);
    auto itrees(trees_.begin());

    for (auto ctrees(itrees + 1), etrees(trees_.end());
         ctrees != etrees; ++itrees, ++ctrees, ++lod)
    {
        LOG(debug) << "gd: " << lod << " -> " << (lod + 1);

        // parent mask is merged-in into child, ignore its size
        ctrees->merge(*itrees, filter);
    }

    return *this;

}

TileIndex& TileIndex::growDown(Flag::value_type type)
{
    auto filter([type](QTree::value_type value) { return (value & type); });

    if (trees_.empty()) { return *this; }

    // traverse trees top to bottom
    auto lod(lodRange().min);
    auto itrees(trees_.begin());

    for (auto ctrees(itrees + 1), etrees(trees_.end());
         itrees != etrees; ++itrees, ++lod)
    {
        LOG(debug) << "gd: " << lod << " -> " << (lod + 1);

        if (ctrees != etrees) {
            // parent mask is merged-in into child, ignore its size
            ctrees->merge(*itrees, filter);

            // increment here, otherwise ctrees would point after end
            ++ctrees;
        }

        // coarsen this node to add siblings
        itrees->coarsen(filter);
    }

    return *this;
}

TileIndex& TileIndex::round(Flag::value_type type)
{
    auto filter([type](QTree::value_type value) { return (value & type); });

    if (trees_.empty()) { return *this; }

    // traverse trees top to bottom
    auto lod(lodRange().min);
    auto itrees(trees_.begin());

    for (auto &tree : trees_) {
        LOG(debug) << "qc: " << lod;

        // coarsen this node to add siblings
        tree.coarsen(filter);
        ++lod;
    }

    return *this;
}

TileIndex& TileIndex::invert(Flag::value_type type)
{
    auto translate([type](QTree::value_type value) -> QTree::value_type {
            return (value & type) ? 0 : type;
        });

    // invert all trees
    for (auto itrees(trees_.begin()), etrees(trees_.end());
         itrees != etrees; ++itrees)
    {
        itrees->translateEachNode(translate);
    }

    return *this;
}

TileIndex& TileIndex::simplify(Flag::value_type type)
{
    auto filter([type](QTree::value_type value) { return (value & type); });

    // invert all trees
    for (auto itrees(trees_.begin()), etrees(trees_.end());
         itrees != etrees; ++itrees)
    {
        itrees->simplify(filter);
    }

    return *this;
}

TileIndex& TileIndex::complete(Flag::value_type type)
{
    auto filter([type](QTree::value_type value) { return (value & type); });

    if (trees_.size() < 2) {
        // nothing to grow
        return *this;
    }

    // traverse trees bottom to top and coarsen -> propagates tiles from bottom
    // to top
    {
        auto lod(lodRange().max);
        auto ctrees(trees_.rbegin());

        for (auto itrees(ctrees + 1), etrees(trees_.rend());
             itrees != etrees; ++itrees, ++ctrees, --lod)
        {
            LOG(debug) << "gu: " << lod << " -> " << (lod - 1);

            // make copy of child
            auto child(*ctrees);
            auto &tree(*itrees);

            // coarsen child (do not change child!)
            child.coarsen(filter);
            // merge in coarsened child -> all parents are set
            tree.merge(child, filter);
        }
    }

    return *this;
}

TileIndex& TileIndex::unset(Flag::value_type type)
{
    if (!type) { return *this; }

    // unset all occurences of type
    auto translator
        ([type](QTree::value_type value) { return (value & ~type); });

    for (auto &tree : trees_) {
        tree.translateEachNode(translator);
    }

    return *this;
}

TileRange TileIndex::Stat::computeTileRange() const
{
    if (lodRange.empty()) { return TileRange(math::InvalidExtents{}); }

    // grab bottommost lod
    TileRange range(tileRanges.back());

    // process all others
    for (auto irange(std::next(tileRanges.rbegin()))
             , erange(tileRanges.rend()); irange != erange; ++irange)
    {
        // make parent range
        range = parent(range);

        // and update
        math::update(range, irange->ll);
        math::update(range, irange->ur);
    }

    // done
    return range;
}

std::pair<LodRange, TileRange> TileIndex::ranges(QTree::value_type mask)
    const
{
    auto stat(statMask(mask));
    return { stat.lodRange, stat.computeTileRange() };
}

void TileIndex::set(const LodRange &lodRange, const TileRange &range
                    , QTree::value_type value)
{
    TileRange r(range);
    for (auto lod : lodRange) {
        if (auto *m = tree(lod, value)) {
            m->set(r.ll(0), r.ll(1), r.ur(0), r.ur(1), value);
        }
        // move to child range
        r = childRange(r);
    }
}

bool TileIndex::validSubtree(Lod lod, const TileId &tileId) const
{
    if (trees_.empty()) { return false; }

    // check all tileindex layers from tileId's lod to the bottom
    for (auto elod(maxLod()); lod <= elod; ++lod) {
        if (auto *m = tree(lod)) {
            // existing layer -> check if there is something in the layer's tree
            // trimmed to tileId's lod depth
            if (m->get(tileId.lod, tileId.x, tileId.y)) {
                return true;
            }
        }
    }
    return false;
}

bool TileIndex::validSubtree(const TileId &tileId) const
{
    return validSubtree(tileId.lod, tileId);
}

TileIndex& TileIndex::shrinkAndComplete(unsigned int trim)
{
    if (empty()) { return *this; }

    auto applyTrim([&](Lod l) { return (l > trim) ? (l - trim) : 0; });
    auto any([&](QTree::value_type value) { return value; });

    // grab last tree and shrink it by trim-levels
    auto lod(lodRange().max);
    auto ctrees(trees_.rbegin());
    ctrees->shrink(applyTrim(lod));
    --lod;

    // process lods in reverse order
    for (auto itrees(ctrees + 1), etrees(trees_.rend());
         itrees != etrees; ++itrees, ++ctrees, --lod)
    {
        auto &tree(*itrees);

        // shrink
        tree.shrink(applyTrim(lod));

        // and make complete by merging-in lower level

        // make copy of child
        auto child(*ctrees);

        // coarsen child (do not change child!)
        child.coarsen(any);

        // merge in coarsened child -> all parents are set
        tree.merge(child, any);
    }

    // done
    return *this;
}

TileIndex& TileIndex::makeAvailable(const LodRange &lodRange)
{
    for (auto lod : lodRange) {
        // force tree creation
        tree(lod, true);
    }

    // done
    return *this;
}

TileIndex& TileIndex::makeAbsolute()
{
    return makeAvailable(LodRange(0, maxLod()));
}


bool TileIndex::check(const boost::filesystem::path &path)
{
    std::ifstream f;

    f.open(path.string(), std::ios_base::in | std::ifstream::binary);
    f.peek();
    if (!f) { return false; }

    return checkMagic(f);
}

TileRange TileIndex::tileRange(Lod lod, QTree::value_type mask) const
{
    TileRange tr(math::InvalidExtents{});

    const auto *t(tree(lod));
    if (!t) { return tr; }

    t->forEachNode([&](unsigned int x, unsigned int y, unsigned int size
                       , QTree::value_type v)
    {
        if (!(v & mask)) { return; }

        // update tile range
        math::update(tr, x, y);
        math::update(tr, x + size - 1, y + size - 1);
    });

    return tr;
}

} } // namespace vadstena::vts
