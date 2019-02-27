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
#include "dbglog/dbglog.hpp"

#include "utility/binaryio.hpp"

#include "../storage/error.hpp"
#include "qtree.hpp"

namespace fs = boost::filesystem;
namespace bin = utility::binaryio;

// uncomment to enable save/load debug output
// #define ENABLE_QTREE_DEBUG_IO

#ifdef ENABLE_QTREE_DEBUG_IO
#  undef QTREE_DEBUG_IO
#  define QTREE_DEBUG_IO(...) __VA_ARGS__
#else
#  undef QTREE_DEBUG_IO
#  define QTREE_DEBUG_IO(...)
#endif

namespace vtslibs { namespace vts {

namespace detail {
    enum : QTree::value_type { GrayNode = 0xff };
} // namespace detail

QTree::QTree(unsigned int order, value_type value)
    : order_(order), size_(1 << order), root_(value)
    , count_(value ? (size_ * size_) : 0), allSetFlags_()
{
}

QTree::QTree(const QTree &other)
    : order_(other.order_), size_(other.size_), root_(other.root_)
    , count_(other.count_), allSetFlags_()
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
        return get(x >> (depth - order_), y >> (depth - order_));
    }

    // calculate size in of a trimmed tree
    unsigned int size(1 << depth);

    if ((x >= size) || (y >= size)) { return 0; }
    return root_.get(size >> 1, x, y);
}

void QTree::set(unsigned int x, unsigned int y, value_type value)
{
    // size check and value
    if ((x >= size_) || (y >= size_)) {
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
            long removedCount(0);
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
        if (old && !value) { return -long(size * size); }
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

const char MAGIC_OLD[2] = { 'Q', 'T' };
const char MAGIC[2] = { 'q', 't' };

const char RASTERMASK_MAGIC_PREFIX[2] = { 'Q', 'M' };
const char RASTERMASK_MAGIC_SUFFIX[3] = { 'A', 'S', 'K' };

namespace limits {
std::uint32_t byte(1 << 8);
std::uint32_t word(1 << 16);
} // namespace limits

namespace node_flags {

constexpr std::uint8_t children(0x0);
constexpr std::uint8_t byte(0x1);
constexpr std::uint8_t word(0x2);
constexpr std::uint8_t dword(0x3);
constexpr std::uint8_t mask(0x3);
constexpr int shift(2);

constexpr std::uint8_t black(0x1);
constexpr std::uint8_t white(0x2);

constexpr std::uint8_t bw(0x80);

inline std::uint8_t flags(std::uint32_t value)
{
    if (value < limits::byte) { return byte; }
    if (value < limits::word) { return word; }
    return dword;
}

inline std::uint8_t flagsBw(std::uint32_t value)
{
    return value ? white : black;
}

inline std::uint8_t type(std::uint8_t flags)
{
    return flags & mask;
}

} // namespace node_flags

inline void saveValue(std::ostream &os, std::uint32_t value)
{
    // write value based on limits
    if (value < limits::byte) {
        QTREE_DEBUG_IO
            (LOG(debug) << "Saving value:  " << std::bitset<8>(value));
        bin::write(os, std::uint8_t(value));
    } else if (value < limits::word) {
        QTREE_DEBUG_IO
            (LOG(debug) << "Saving value:  " << std::bitset<16>(value));
        bin::write(os, std::uint16_t(value));
    } else {
        QTREE_DEBUG_IO
            (LOG(debug) << "Saving value:  " << std::bitset<32>(value));
        bin::write(os, std::uint32_t(value));
    }
}

template <typename T>
inline QTree::value_type loadValue(std::istream &is)
{
    T value;
    bin::read(is, value);
    QTREE_DEBUG_IO
        (LOG(debug) << "Loading value: " << std::bitset<sizeof(T) * 8>(value));
    return value;
}

} // namespace

void QTree::save(std::ostream &os, const SaveParams &params) const
{
    // header
    bin::write(os, MAGIC);
    bin::write(os, std::uint16_t(2));
    QTREE_DEBUG_IO
        (LOG(debug) << "magic: " << MAGIC[0] << ", " << MAGIC[1]
         << " (" << os.tellp() << ")");
    bin::write(os, std::uint8_t(order_));

    std::uint8_t baseFlags(params.bw() ? node_flags::bw : 0);

    // write this node
    if (root_.children) {
        // node has children -> save children
        bin::write(os, std::uint8_t(baseFlags));
        QTREE_DEBUG_IO
            (LOG(debug) << "Write flags: " << std::bitset<8>(baseFlags));
        if (params.bw()) {
            root_.saveChildrenBw(os);
        } else {
            root_.saveChildren(os);
        }
    } else {
        if (params.bw()) {
            const auto flags(baseFlags | node_flags::flagsBw(root_.value));
            QTREE_DEBUG_IO
                (LOG(debug) << "Write flags: " << std::bitset<8>(flags));
            bin::write(os, std::uint8_t(flags));
        } else {
            const auto flags(baseFlags | node_flags::flags(root_.value));
            QTREE_DEBUG_IO
                (LOG(debug) << "Write flags: " << std::bitset<8>(flags));
            bin::write(os, std::uint8_t(flags));
            saveValue(os, root_.value);
        }
    }
}

namespace {

std::uint8_t size2order(int size)
{
    std::uint8_t order(0);
    while (size != 1) {
        order++;
        size >>= 1;
    }
    return order;
}

} // namesapce

void QTree::load(std::istream &is, const fs::path &path)
{
    // read and check magic
    char magic[sizeof(MAGIC)];
    bin::read(is, magic);

    std::uint16_t version(0);
    if (!std::memcmp(magic, MAGIC, sizeof(MAGIC))) {
        // new format, version follows
        bin::read(is, version);
    } else if (!std::memcmp(magic, MAGIC_OLD, sizeof(MAGIC_OLD))) {
        // old format, 1
        version = 1;
    } else if (!std::memcmp(magic, RASTERMASK_MAGIC_PREFIX, sizeof(MAGIC))) {
        // seems like old plain raster mask -> load
        char magic2[sizeof(RASTERMASK_MAGIC_SUFFIX)];
        bin::read(is, magic2);
        if (std::memcmp(magic2, RASTERMASK_MAGIC_SUFFIX
                        , sizeof(RASTERMASK_MAGIC_SUFFIX)))
        {
            // hm, but is not :P
            LOGTHROW(err1, storage::BadFileFormat)
                << "File " << path << " is neither a VTS qtree "
                "nor a imgproc raster mask file.";
        }

        uint8_t dummy;
        bin::read(is, dummy); // reserved
        bin::read(is, dummy); // reserved
        bin::read(is, dummy); // reserved

        // ignore dimensions and count, use quad size to compute order
        std::int32_t x, y, qs;
        bin::read(is, x);
        bin::read(is, y);
        bin::read(is, qs);
        std::uint32_t count;
        bin::read(is, count);

        // compute order
        std::uint8_t order(size2order(qs));
        recreate(order);
        std::tie(count_, allSetFlags_) = root_.loadRasterMask(size_, is);
        return;
    } else {
        LOGTHROW(err1, storage::BadFileFormat)
            << "File " << path << " is not a VTS qtree file.";
    }

    switch (version) {
    case 1: {
        std::uint8_t order;
        bin::read(is, order);
        recreate(order);
        std::tie(count_, allSetFlags_) = root_.loadV1(size_, is);
    } break;

    case 2: {
        QTREE_DEBUG_IO
            (LOG(debug) << "magic: " << magic[0] << ", " << magic[1]
             << " (" << is.tellg() << ")");

        std::uint8_t order;
        bin::read(is, order);
        recreate(order);
        std::uint8_t flags;
        bin::read(is, flags);
        QTREE_DEBUG_IO
            (LOG(debug) << "Read flags:  " << std::bitset<8>(flags));

        if (flags & node_flags::bw) {
            // process root node
            std::tie(count_, allSetFlags_) = root_.loadBw(size_, is, flags);
        } else {
            // process root node
            std::tie(count_, allSetFlags_) = root_.load(size_, is, flags);
        }
    } break;

    default:
        LOGTHROW(err1, storage::BadFileFormat)
            << "File " << path << " invalid VTS qtree file version <"
            << version << ">.";
    }
}

void QTree::Node::saveChildren(std::ostream &os)
    const
{
    // collect and write child flags
    std::uint8_t flags(0);
    int shift(0);
    for (const auto &node : children->nodes) {
        if (!node.children) {
            flags |= node_flags::flags(node.value) << shift;
        }
        shift += node_flags::shift;
    }
    QTREE_DEBUG_IO
        (LOG(debug) << "Write flags: " << std::bitset<8>(flags));
    bin::write(os, flags);

    // write children
    for (const auto &node : children->nodes) {
        if (node.children) {
            // node has children -> save children
            node.saveChildren(os);
        } else {
            // we have to write actual value
            saveValue(os, node.value);
        }
    }
}

void QTree::Node::saveChildrenBw(std::ostream &os)
    const
{
    // collect and write child flags
    std::uint8_t flags(0);
    int shift(0);
    for (const auto &node : children->nodes) {
        if (!node.children) {
            flags |= node_flags::flagsBw(node.value) << shift;
        }
        shift += node_flags::shift;
    }
    QTREE_DEBUG_IO
        (LOG(debug) << "Write flags: " << std::bitset<8>(flags));
    bin::write(os, flags);

    // write children
    for (const auto &node : children->nodes) {
        if (node.children) {
            // node has children -> save children
            node.saveChildrenBw(os);
        }
    }
}

std::tuple<std::size_t, QTree::value_type>
QTree::Node::loadV1(unsigned int mask, std::istream &is)
{
    std::uint8_t u8;
    bin::read(is, u8);
    if (u8 == detail::GrayNode) {
        children.reset(new Children());
        std::tuple<std::size_t, value_type> res(0, 0);
        for (auto &node : children->nodes) {
            const auto sub(node.loadV1(mask >> 1, is));
            std::get<0>(res) += std::get<0>(sub);
            std::get<1>(res) |= std::get<1>(sub);
        }
        return res;
    }

    value = u8;
    return std::tuple<std::size_t, value_type>
        ((value > 0) * (mask * mask), value);
}

std::tuple<std::size_t, QTree::value_type>
QTree::Node::load(unsigned int mask, std::istream &is
                  , std::uint8_t flags)
{
    switch (node_flags::type(flags)) {
    case node_flags::children:
        // handle node's children
        children.reset(new Children());
        return loadChildren(mask >> 1, is);

    case node_flags::byte:
        value = loadValue<std::uint8_t>(is);
        break;

    case node_flags::word:
        value = loadValue<std::uint16_t>(is);
        break;

    case node_flags::dword:
        value = loadValue<std::uint32_t>(is);
        break;
    }

    return std::tuple<std::size_t, value_type>
        ((value > 0) * (mask * mask), value);
}

std::tuple<std::size_t, QTree::value_type>
QTree::Node::loadBw(unsigned int mask, std::istream &is
                    , std::uint8_t flags)
{
    switch (node_flags::type(flags)) {
    case node_flags::children:
        // handle node's children
        children.reset(new Children());
        return loadChildrenBw(mask >> 1, is);

    case node_flags::black:
        // black -> zero
        value = 0;
        break;

    default:
        // other -> one
        value = 1;
        break;
    }

    return std::tuple<std::size_t, value_type>
        ((value > 0) * (mask * mask), value);
}

std::tuple<std::size_t, QTree::value_type>
QTree::Node::loadChildren(unsigned int mask, std::istream &is)
{
    std::tuple<std::size_t, value_type> statistics;

    std::uint8_t flags;
    bin::read(is, flags);
    QTREE_DEBUG_IO
        (LOG(debug) << "Read flags:  " << std::bitset<8>(flags));
    for (auto &node : children->nodes) {
        // process node
        const auto sub(node.load(mask, is, flags));

        // update statistics
        std::get<0>(statistics) += std::get<0>(sub);
        std::get<1>(statistics) |= std::get<1>(sub);

        // next node flags
        flags >>= node_flags::shift;
    }

    return statistics;
}

std::tuple<std::size_t, QTree::value_type>
QTree::Node::loadChildrenBw(unsigned int mask, std::istream &is)
{
    std::tuple<std::size_t, value_type> statistics;

    std::uint8_t flags;
    bin::read(is, flags);
    QTREE_DEBUG_IO
        (LOG(debug) << "Read flags:  " << std::bitset<8>(flags));
    for (auto &node : children->nodes) {
        // process node
        const auto sub(node.loadBw(mask, is, flags));

        // update statistics
        std::get<0>(statistics) += std::get<0>(sub);
        std::get<1>(statistics) |= std::get<1>(sub);

        // next node flags
        flags >>= node_flags::shift;
    }

    return statistics;
}

std::tuple<std::size_t, QTree::value_type>
QTree::Node::loadRasterMask(unsigned int mask, std::istream &is)
{
    std::uint8_t u8;
    bin::read(is, u8);
    switch (u8) {
    case 0: // white (really)
        value = 1;
        return std::tuple<std::size_t, value_type>((mask * mask), 1);

    case 1: // black (no kidding)
        value = 0;
        return std::tuple<std::size_t, value_type>(0, 0);

    default: // gray node
        break;
    }

    // gray node
    children.reset(new Children());
    std::tuple<std::size_t, value_type> res(0, 0);
    for (auto &node : children->nodes) {
        auto sub(node.loadRasterMask(mask >> 1, is));
        std::get<0>(res) += std::get<0>(sub);
        std::get<1>(res) += std::get<1>(sub);
    }

    // done
    return res;
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
               << " to " << depth << ".";
    // sanity check
    if (depth > order_) { return; }

    root_.shrink(depth);

    // update dimensions
    order_ = depth;
    size_ = 1 << order_;

    // recount
    recount();
}

void QTree::Node::shrink(unsigned int depth)
{
    if (!children) {
        value = bool(value);
        return;
    }

    if (!depth) {
        // contract inner node to just one leaf
        children.reset();
        value = 1;
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

QTree::QTree(const QTree &other, unsigned int order
             , unsigned int depth, unsigned int x, unsigned int y)
    : order_(order), size_(1 << order), root_(other.root_.find(depth, x, y))
    , count_(0), allSetFlags_()
{
    recount();
}

/** Finds quad in given subtree.
 */
const QTree::Node& QTree::Node::find(unsigned int depth
                                     , unsigned int x, unsigned int y)
    const
{
    // if we can descend down (i.e. both tree and depth allow)
    if (depth && children) {
        // find node to descend
        --depth;
        unsigned int mask(1 << depth);

        auto index(((x & mask) ? 1 : 0) | ((y & mask) ? 2 : 0));
        return children->nodes[index].find(depth, x, y);
    }

    // there is nothing more down there
    return *this;
}

void QTree::force(value_type value)
{
    root_.force(value);
}

void QTree::Node::force(value_type newValue)
{
    if (children) {
        // descend
        children->nodes[0].force(newValue);
        children->nodes[1].force(newValue);
        children->nodes[2].force(newValue);
        children->nodes[3].force(newValue);
        contract();
        return;
    }

    // rewrite
    if (value) { value = newValue; }
}

} } // namespace vtslibs::vts
