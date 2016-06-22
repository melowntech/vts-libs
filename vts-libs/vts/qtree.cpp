#include "dbglog/dbglog.hpp"

#include "utility/binaryio.hpp"

#include "../storage/error.hpp"
#include "./qtree.hpp"

namespace fs = boost::filesystem;
namespace bin = utility::binaryio;

namespace vadstena { namespace vts {

namespace detail {
    enum : QTree::value_type { GrayNode = 0xff };
    enum : QTree::value_type { GrayNodeReplacement = 0x7f };
} // namespace detail

QTree::QTree(unsigned int order, value_type value)
    : order_(order), size_(1 << order), root_(value)
    , count_(value ? (size_ * size_) : 0)
{
}

QTree::QTree(const QTree &other)
    : order_(other.order_), size_(other.size_), root_(other.root_)
    , count_(other.count_)
{}

QTree& QTree::operator=(const QTree &other)
{
    if (&other != this) {
        order_ = other.order_;
        size_ = other.size_;
        root_ = other.root_;
        count_ = other.count_;
    }
    return *this;
}

void QTree::recreate(unsigned int order, value_type value)
{
    // create new tree
    *this = QTree(order, value);
}

QTree::value_type QTree::get(unsigned int x, unsigned int y) const
{
    // size check
    if ((x >= size_) || (y >= size_)) { return 0; }
    return root_.get(size_ >> 1, x, y);
}

QTree::value_type QTree::get(unsigned int depth
                             , unsigned int x, unsigned int y)
    const
{
    // not trimming -> regular get
    if (depth >= order_) {
        // too deep, move up
        get(x >> (depth - order_), y >> (depth - order_));
    }

    // calculate size in of a trimmed tree
    unsigned int size(1 << depth);

    if ((x >= size) || (y >= size)) { return 0; }
    return root_.get(size >> 1, x, y);
}

void QTree::set(unsigned int x, unsigned int y, value_type value)
{
    // size check and value
    if ((x >= size_) || (y >= size_) || (value == detail::GrayNode)) {
        return;
    }
    count_ += root_.set(size_ >> 1, x, y, value);
}

void QTree::set(unsigned int x1, unsigned int y1
                , unsigned int x2, unsigned int y2, value_type value)
{
    // NB: left/top boundary not checked since we use unsigned numbers

    // check for block completely outside the pane
    if ((x1 >= size_) || (y1 >= size_)) { return; }
    if (value == detail::GrayNode) { return; }

    // clip to pane
    if (x2 >= size_) { x2 = size_ - 1; }
    if (y2 >= size_) { y2 = size_ - 1; }

    // and go down
    count_ += root_.set(size_, 0, 0, x1, y1, x2, y2, value);
}

void QTree::reset(value_type value)
{
    root_ = Node(value);
}

QTree::Node::Node(const Node &other)
    : value(other.value), children(other.children
                                   ? new Children(*other.children)
                                   : nullptr)
{}

QTree::Node& QTree::Node::operator=(const Node &other)
{
    if (&other != this) {
        value = other.value;
        children.reset
            (other.children ? new Children(*other.children) : nullptr);
    }
    return *this;
}

QTree::Node::~Node() {}

QTree::value_type QTree::Node::get(unsigned int mask, unsigned int x
                                   , unsigned int y) const
{
    if (!children) {
        // no children -> get value of this node
        return value;
    } else if (!mask) {
        // too deep
        return ~value_type(0);
    }

    // find child and go there
    auto index(((x & mask) ? 1 : 0) | ((y & mask) ? 2 : 0));
    return children->nodes[index].get(mask >> 1, x, y);
}

long QTree::Node::set(unsigned int mask, unsigned int x, unsigned int y
                      , value_type value)
{
    if (!mask) {
        // bottom of the tree -> we can change this one
        auto old(this->value);
        this->value = value;
        if ((old && value) || (!old && !value)) { return 0; }
        if (old && !value) { return -1; }
        return 1;
    }

    // not at the bottom of the tree
    if (!children) {
        if (this->value == value) {
            // no-op
            return 0;
        }

        // split
        children.reset(new Children(this->value));
    }

    // we have children here, descend
    auto index(((x & mask) ? 1 : 0) | ((y & mask) ? 2 : 0));
    auto res(children->nodes[index].set(mask >> 1, x, y, value));
    contract();

    return res;
}

long QTree::Node::set(unsigned int size, unsigned int x, unsigned int y
                      , unsigned int x1, unsigned int y1
                      , unsigned int x2, unsigned int y2, value_type value)
{
    if ((x2 < x) || (x1 >= (x + size))
        || (y2 < y) || (y1 >= (y + size)))
    {
        // outside of given range -> nothing to do
        return 0;
    }

    if ((x1 <= x) && (x2 >= (x + size - 1))
        && (y1 <= y) && (y2 >= (y + size - 1)))
    {
        // we are inside given range, set full value
        if (children) {
            // accumulate area to destroy
            std::size_t removedCount(0);
            descend(size, x, y, [&](unsigned int, unsigned int
                                    , unsigned int size, value_type)
            {
                removedCount += size * size;
            }, Filter::white);

            // destroy children
            children.reset();

            // set new value
            this->value = value;

            if (value) {
                // setting to value, set value is full node without removed
                // count
                return (size * size) - removedCount;
            }

            // setting to no value -> everything was removed
            return -removedCount;
        }

        // this is just one node, almost same as one pixel setting
        auto old(this->value);
        this->value = value;
        if ((old && value) || (!old && !value)) { return 0; }
        if (old && !value) { return -(size * size); }
        return (size * size);
    }

    // not at the bottom of the tree, split into four child nodes
    if (!children) {
        if (this->value == value) {
            // no-op
            return 0;
        }

        // split
        children.reset(new Children(this->value));
    }

    // go down to all four children
    size >>= 1;
    long res(0);
    res += children->nodes[0].set
        (size, x, y, x1, y1, x2, y2, value);
    res += children->nodes[1].set
        (size, x + size, y, x1, y1, x2, y2, value);
    res += children->nodes[2].set
        (size, x, y + size, x1, y1, x2, y2, value);
    res += children->nodes[3].set
        (size, x + size, y + size, x1, y1, x2, y2, value);

    // contract nodes of same value
    contract();
    return res;
}

bool QTree::Node::Children::sameValue() const
{
    // grab first node's value
    auto value(nodes.front().value);
    // check all nodes for having same value and no children
    for (const auto &node : nodes) {
        if (node.children || (node.value != value)) {
            return false;
        }
    }

    // all nodes are the same
    return true;
}

void QTree::Node::contract()
{
    // childless -> nothing to do
    if (!children) { return; }

    if (children->sameValue()) {
        // all 4 children has the same color -> destroy and replace with this
        // ndoe
        value = children->nodes[0].value;
        children.reset();
    }
}

namespace {
    const char MAGIC[2] = { 'Q', 'T' };
}

void QTree::save(std::ostream &os) const
{
    bin::write(os, MAGIC);
    bin::write(os, std::uint8_t(order_));

    root_.save(os);
}

void QTree::load(std::istream &is, const fs::path &path)
{
    // read and check magic
    char magic[sizeof(MAGIC)];
    bin::read(is, magic);

    if (std::memcmp(magic, MAGIC, sizeof(MAGIC))) {
        LOGTHROW(err1, storage::BadFileFormat)
            << "File " << path << " is not a VTS mesh file.";
    }

    std::uint8_t order;
    bin::read(is, order);
    recreate(order);
    std::tie(count_, allSetFlags_) = root_.load(size_, is);
}

void QTree::Node::save(std::ostream &os) const
{
    if (children) {
        // mark as gray node
        bin::write(os, std::uint8_t(detail::GrayNode));
        for (const auto &node : children->nodes) { node.save(os); }
    } else {
        // just value
        std::uint8_t u8(value);
        if (u8 == detail::GrayNode) {
            u8 = detail::GrayNodeReplacement;
        }
        bin::write(os, u8);
    }
}

std::tuple<std::size_t, QTree::value_type>
QTree::Node::load(unsigned int mask, std::istream &is)
{
    std::uint8_t u8;
    bin::read(is, u8);
    if (u8 == detail::GrayNode) {
        children.reset(new Children());
        std::tuple<std::size_t, value_type> res(0, 0);
        for (auto &node : children->nodes) {
            auto sub(node.load(mask >> 1, is));
            std::get<0>(res) += std::get<0>(sub);
            std::get<1>(res) += std::get<1>(sub);
        }
        return res;
    }

    value = u8;
    return std::tuple<std::size_t, value_type>
        ((value > 0) * (mask * mask), value);
}

void QTree::recount()
{
    std::size_t count(0);
    forEachNode([&](unsigned int, unsigned int, unsigned int size, value_type)
    {
        count += size * size;
    }, Filter::white);
    count_ = count;
}

void QTree::shrink(unsigned int depth)
{
    LOG(info1) << "Shrinking tree with order " << order_
               << " to " << depth;
    // sanity check
    if (depth >= order_) { return; }

    root_.shrink(depth);

    // update dimensions
    order_ = depth;
    size_ = 1 << order_;

    // recount
    recount();
}

void QTree::Node::shrink(unsigned int depth)
{
    if (!children) { return; }

    if (!depth) {
        // contract inner node to just one leaf
        children.reset();
        value = ~value_type(0);
        return;
    }

    // non leaf -> descend
    --depth;
    children->nodes[0].shrink(depth);
    children->nodes[1].shrink(depth);
    children->nodes[2].shrink(depth);
    children->nodes[3].shrink(depth);

    // contract if possible
    contract();
}

} } // namespace vadstena::vts
