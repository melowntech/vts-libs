#include "dbglog/dbglog.hpp"

#include "utility/binaryio.hpp"

#include "../storage/error.hpp"
#include "./qtree.hpp"

namespace fs = boost::filesystem;
namespace bin = utility::binaryio;

namespace vadstena { namespace vts {

namespace detail {
    enum : QTree::value_type { GrayNode = 0xff };
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

void QTree::set(unsigned int x, unsigned int y, value_type value)
{
    // size check and value
    if ((x >= size_) || (y >= size_) || (value == detail::GrayNode)) {
        return;
    }
    count_ += root_.set(size_ >> 1, x, y, value);
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
        if (old && !value) { return -(mask * mask); }
        return (mask * mask);
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
    count_ = root_.load(size_, is);
}

void QTree::Node::save(std::ostream &os) const
{
    if (children) {
        // mark as gray node
        bin::write(os, std::uint8_t(detail::GrayNode));
        for (const auto &node : children->nodes) { node.save(os); }
    } else {
        // just value
        bin::write(os, std::uint8_t(value));
    }
}

std::size_t QTree::Node::load(unsigned int mask, std::istream &is)
{
    std::uint8_t u8;
    bin::read(is, u8);
    if (u8 == detail::GrayNode) {
        children.reset(new Children());
        std::size_t count(0);
        for (auto &node : children->nodes) {
            count += node.load(mask >> 1, is);
        }
        return count;
    }

    value = u8;
    return (value > 0) * (mask * mask);
}

void QTree::merge(const QTree &other, bool checkDimensions)
{
    if (checkDimensions) {
        if (order_ != other.order_) {
            LOGTHROW(err1, std::runtime_error)
                << "Attempt to merge in data from qtree with diferent order.";
        }
    }

    root_.merge(other.root_);
    recount();
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

void QTree::Node::merge(const Node &other)
{
    if ((value) || (!other.value)) {
        // merge(WHITE, anything) = WHITE (keep)
        // merge(anything, BLACK) = anything (keep)
        return;
    }

    if (other.value) {
        // merge(anything, WHITE) = WHITE
        *this = other;
        return;
    }

    // OK, other is gray
    if (!value) {
        // merge(BLACK, GRAY) = GRAY
        *this = other;
        return;
    }

    // merge(GRAY, GRAY) = go down
    children->nodes[0].merge(other.children->nodes[0]);
    children->nodes[1].merge(other.children->nodes[1]);
    children->nodes[2].merge(other.children->nodes[2]);
    children->nodes[3].merge(other.children->nodes[3]);

    // contract if possible
    contract();
}

} } // namespace vadstena::vts
