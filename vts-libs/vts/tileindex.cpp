#include <boost/filesystem.hpp>
#include <boost/format.hpp>

#include <opencv2/highgui/highgui.hpp>

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
{
}

TileIndex::TileIndex(const TileIndex &other, ShallowCopy)
    : minLod_(other.minLod_)
{
    trees_.reserve(other.trees_.size());
    for (const auto &tree : other.trees_) {
        trees_.emplace_back(tree.order());
    }
}

TileIndex::TileIndex(LodRange lodRange
                     , const TileIndex *other, bool noFill)
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

void TileIndex::load(std::istream &f, const fs::path &path)
{
    using utility::binaryio::read;

    char magic[sizeof(TILE_INDEX_IO_MAGIC)];
    read(f, magic);

    if (std::memcmp(magic, TILE_INDEX_IO_MAGIC, sizeof(TILE_INDEX_IO_MAGIC))) {
        LOGTHROW(err2, storage::Error)
            << "TileIndex " << path << " has wrong magic.";
    }

    uint8_t minLod, size;
    read(f, minLod);
    read(f, size);
    minLod_ = minLod;

    trees_.resize(size);

    for (auto &tree : trees_) {
        tree.load(f, path);
    }
}

void TileIndex::load(const fs::path &path)
{
    std::ifstream f;
    f.exceptions(std::ifstream::failbit | std::ifstream::badbit);
    f.open(path.string(), std::ifstream::in | std::ifstream::binary);

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
                        , bool on)
{
    update(tileId, [&](QTree::value_type value) -> QTree::value_type
    {
        return (on
                ? (value | mask)
                : (value & ~mask));
    });
}

TileIndex::Stat TileIndex::statMask(QTree::value_type mask) const
{
    Stat stat;

    auto lod(minLod_);
    for (const auto &tree : trees_) {
        stat.tileRanges.emplace_back(math::InvalidExtents{});
        auto &tileRange(stat.tileRanges.back());

        tree.forEachNode([&](unsigned int x, unsigned int y, unsigned int size
                             , QTree::value_type value)
        {
            if (!(value & mask)) { return; }

            // remember lod
            storage::update(stat.lodRange, lod);

            // update tile range
            math::update(tileRange, x, y);
            math::update(tileRange, x + size - 1, y + size - 1);
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

TileIndex& TileIndex::growUp(Flag::value_type type)
{
    auto filter([type](QTree::value_type value) { return (value & type); });

    if (trees_.size() < 2) {
        // nothing to grow
        return *this;
    }

    // traverse trees bottom to top
    auto lod(lodRange().max);
    auto ctrees(trees_.rbegin());

    for (auto itrees(ctrees + 1), etrees(trees_.rend());
         itrees != etrees; ++itrees, ++ctrees, --lod)
    {
        LOG(debug) << "gu: " << lod << " -> " << (lod - 1);

        auto &child(*ctrees);

        // coarsen child tree (kill all 1-pixel nodes) -> each tile has its
        // sibling
        child.coarsen(filter);

        // and merge this coarsened child into parent tree -> each tile gets
        // parent
        itrees->merge(child, filter);
    }

    return *this;
}

TileIndex& TileIndex::growDown(Flag::value_type type)
{
    auto filter([type](QTree::value_type value) { return (value & type); });

    if (trees_.size() < 2) {
        // nothing to grow
        return *this;
    }

    // traverse trees top to bottom
    auto lod(lodRange().min);
    auto ptrees(trees_.begin());

    for (auto itrees(ptrees + 1), etrees(trees_.end());
         itrees != etrees; ++itrees, ++ptrees, ++lod)
    {
        LOG(debug) << "gd: " << lod << " -> " << (lod + 1);

        // merge in parent mask, ignore its size
        itrees->merge(*ptrees, filter);
    }

    return *this;
}

TileIndex& TileIndex::invert(Flag::value_type type)
{
    auto translate([type](QTree::value_type value) {
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

namespace {

double pixelSize(const math::Size2 &dims, const long maxArea)
{
    long a(long(dims.width) * long(dims.height));
    auto scale(std::sqrt(double(maxArea) / double(a)));
    return scale;
}

void dumpTree(const fs::path &path, const QTree &tree
              , TileIndex::Flag::value_type type, double pixelSize)
{
    const auto size(tree.size());
    cv::Mat m(long(std::ceil(pixelSize * size.height))
              , long(std::ceil(pixelSize * size.width))
              , CV_8UC1);
    m = cv::Scalar(0);
    // auto white(cv::Scalar(0xff));

    const std::vector<cv::Scalar> colors = { cv::Scalar(0xff)
                                             , cv::Scalar(0xaf)
                                             , cv::Scalar(0x7f)
                                             , cv::Scalar(0x3f) };

    LOG(info4) << tree.size() << ": " << path << ": " << pixelSize
               << " -> " << m.size();

    uint fracAdj(std::round(pixelSize) - pixelSize != 0.0 ? 1 : 0);

    int color(0);

    tree.forEachNode([&](unsigned int x, unsigned int y, unsigned int size
                         , QTree::value_type value)
    {
        if (!(value & type)) { return; }
        cv::Point2i start(int(std::floor(pixelSize * x))
                          , int(std::floor(pixelSize * y)));
        cv::Point2i end
            (int(std::ceil(pixelSize * (x + size - fracAdj )))
             , int(std::ceil(pixelSize * (y + size - fracAdj))));

        LOG(info4) << "quad: " << start << " -> " << end
                   << ", size: " << size;

        cv::rectangle(m, start, end, colors[color], CV_FILLED, 4);
        color = (color + 1) % colors.size();
    }, QTree::Filter::white);

    imwrite(path.string(), m);
}

} // namespace

void dumpAsImages(const fs::path &path, const TileIndex &ti
                  , TileIndex::Flag::value_type type, const long maxArea)
{
    LOG(info2) << "Dumping tileIndex as image stack at " << path << ".";
    create_directories(path);

    if (ti.trees().empty()) { return; }

    auto lod(ti.lodRange().max);
    const auto &trees(ti.trees());

    for (auto itrees(trees.rbegin()), etrees(trees.rend());
         itrees != etrees; ++itrees)
    {
        LOG(info1) << "Dumping lod " << lod;

        // rasterize and dump
        auto file(path / str(boost::format("%02d.png") % lod));
        dumpTree(file, *itrees, type, pixelSize(itrees->size(), maxArea));

        // next level
        --lod;
    }
}

} } // namespace vadstena::vts
