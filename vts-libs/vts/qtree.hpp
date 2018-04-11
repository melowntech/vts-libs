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
#ifndef vtslibs_vts_qtree_hpp_included_
#define vtslibs_vts_qtree_hpp_included_

#include <cstdint>
#include <array>
#include <iostream>
#include <iostream>
#include <functional>

#include <boost/optional.hpp>
#include <boost/filesystem/path.hpp>

#include "dbglog/dbglog.hpp"

#include "math/geometry_core.hpp"

namespace vtslibs { namespace vts {

class QTree {
public:
    typedef std::uint32_t value_type;
    typedef boost::optional<value_type> opt_value_type;

    QTree(unsigned int order = 0, value_type value = 0);

    QTree(const QTree &other);
    QTree& operator=(const QTree &other);

    QTree(QTree&&) = default;
    QTree& operator=(QTree&&) = default;

    /** Initialize qtree from other qtree's subtree at given coordinates.
     */
    QTree(const QTree &other, unsigned int order
          , unsigned int depth, unsigned int x, unsigned int y);

    /** Read value from pixel at (x, y).
     */
    value_type get(unsigned int x, unsigned int y) const;

    /** Read value from pixel from node (x, y) in tree reduced to given depth.
     *  Coordinates (x, y) are interpreted as if they point into trimmed tree.
     *  Returns either value from first valid node or value_type(~0) if internal
     *  node is hit first
     */
    value_type get(unsigned int depth, unsigned int x, unsigned int y) const;

    void set(unsigned int x, unsigned int y, value_type value);

    /** Set given range.
     */
    void set(unsigned int x1, unsigned int y1
             , unsigned int x2, unsigned int y2, value_type value);

    /** Force given value to all non-zero pixels.
     */
    void force(value_type value);

    /** Update given range.
     */
    template <typename Op>
    void update(unsigned int x1, unsigned int y1
                , unsigned int x2, unsigned int y2, Op op);

    void reset(value_type value);

    /** Save parameters.
     */
    class SaveParams {
    public:
        SaveParams() : bw_(false) {}

        inline SaveParams& bw(bool value) { bw_ = value; return *this; }
        inline bool bw() const { return bw_; }

    private:
        bool bw_;
    };

    void save(std::ostream &os, const SaveParams &params = SaveParams()) const;
    void load(std::istream &is, const boost::filesystem::path &path);

    void recreate(unsigned int order = 0, value_type value = 0);

    unsigned int order() const { return order_; }

    /** Merge nodes.
     */
    template <typename FilterOp>
    void merge(const QTree &other, const FilterOp &filter);

    /** Coarsen one level up.
     *
     * \param filter filtering operation
     * \param resize decrement order (halve size) if true
     */
    template <typename FilterOp>
    void coarsen(const FilterOp &filter, bool resize = false);

    /** Coarsen one level up if filter is valid for at least one leaf.
     *
     * \param filter filtering operation
     */
    template <typename FilterOp>
    void conditionalCoarsen(const FilterOp &filter);

    /** Intersect nodes.
     */
    template <typename FilterOp>
    void intersect(const QTree &other, const FilterOp &filter);

    /** Checks for intersection.
     */
    template <typename FilterOp>
    bool overlaps(const QTree &other, const FilterOp &filter) const;

    /** Returns number of non-zero elements.
     */
    std::size_t count() const { return count_; }

    /** Returns true if there is no non-zero element.
     */
    bool empty() const { return !count_; }

    /** Returns true if there is no zero element.
     */
    bool full() const { return count_ == size_ * size_; }

    math::Size2 size() const { return math::Size2(size_, size_); }

    enum class Filter {
        black, white, both
    };

    /** Runs op(x, y, xsize, value) for each node based on filter.
     */
    template <typename Op>
    void forEachNode(const Op &op, Filter filter = Filter::both) const;

    /** Runs op(x, y, value) for each element based on filter.
     *  Uses forEachNode and rasterizes node internally.
     */
    template <typename Op>
    void forEach(const Op &op, Filter filter = Filter::both) const;

    /** Runs op(x, y, value) for each node based on filter in subtree starting
     *  at given index.
     */
    template <typename Op>
    void forEachNode(unsigned int depth, unsigned int x, unsigned int y
                     , const Op &op, Filter filter = Filter::both) const;

    /** Runs op(x, y, value) for each pixel based on filter in subtree starting
     *  at given index.
     */
    template <typename Op>
    void forEach(unsigned int depth, unsigned int x, unsigned int y
                 , const Op &op, Filter filter = Filter::both) const;

    /** Translates value of every node.
     *  Calls value = op(value) for every node.
     */
    template <typename Op>
    void translateEachNode(const Op &op);

    /** Converts to simplified quadrant tree: value is converted to 1 (when
     *  type(filter) says white) or 0 (when type(filter) says black.
     */
    template <typename FilterOp>
    void simplify(const FilterOp &filter);

    /** Incorporates another tree into this one.
     *
     * \param other tree to combine with
     * \param combiner functor taking two node values and returning new value
     */
    template <typename Combiner>
    void combine(const QTree &other, const Combiner &combiner);

    /** Shrinks tree to given depth.
     *  If depth is >= current depth nothing happens.
     *  Original inner nodes become leafs with value_type(~0).
     *
     *  This function is equivalent to reconstructing tree from results of
     *  get(depth, ?, ?).
     */
    void shrink(unsigned int depth);

    /** Return all set flags.
     */
    value_type allSetFlags() const { return allSetFlags_; }

    /** Create qtree from this qtrees's subtree at given coordinates.
     */
    QTree subTree(unsigned int order, unsigned int depth
                  , unsigned int x, unsigned int y) const
    {
        return { *this, order, depth, x, y };
    }

    void swap(QTree &other);

    /** Converter support.
     *
     *  Conterter interface:
     *
     *  // shortcut to optional value
     *  typedef boost::optional<QTree::value_type> opt_value_type;
     *
     *  struct Converter {
     *      // called once for tree root
     *      void root(opt_value_type);
     *
     *      // called with values of all node's children
     *      // invalid value marks non-leaf node
     *      // returned value is sent to individual calls to enter()
     *      any_type children(opt_value_type ul, opt_value_type ur
     *                        , opt_value_type ll, opt_value_type lr);
     *
     *      // called before descending into individual internal node
     *      // any is anything children callback returned
     *      // node index: 0=ul, 1=ur, 2=ll, 3=lr
     *      void ender(const any_type &any, int nodeIndex);
     *
     *      // called on node exit
     *      // any is anything children callback returned
     *      // node index: 0=ul, 1=ur, 2=ll, 3=lr
     *      void leave(const any_type &any, int nodeIndex);
     *  }
     */
    template <typename Converter>
    void convert(Converter &converter) const;

    /** Check that all values stored in the quad tree pass given matcher.
     */
    template <typename Matcher>
    bool matchAll(const Matcher &matcher) const;

    /** Compare this quad tree with another via provided comparator.
     */
    template <typename Comparator>
    bool compare(const QTree &other, const Comparator &comparator) const;

private:
    /** Re-calculates number of non-zero elements.
     */
    void recount();

    struct Node {
        QTree::value_type value;

        struct Children;
        std::unique_ptr<Children> children;

        Node(value_type value = 0) : value(value), children() {}
        Node(const Node &other);
        Node& operator=(const Node &other);
        Node(Node&&) = default;
        Node& operator=(Node&&) = default;

        ~Node();

        value_type get(unsigned int mask, unsigned int x, unsigned int y)
            const;

        /** Returns size change
         */
        long set(unsigned int mask, unsigned int x, unsigned int y
                 , value_type value);

        long set(unsigned int size, unsigned int x, unsigned int y
                 , unsigned int x1, unsigned int y1
                 , unsigned int x2, unsigned int y2, value_type value);

        template <typename Op>
        long update(unsigned int size, unsigned int x, unsigned int y
                    , unsigned int x1, unsigned int y1
                    , unsigned int x2, unsigned int y2, Op op);

        void contract();

        void saveChildren(std::ostream &os) const;
        void saveChildrenBw(std::ostream &os) const;

        /** Load node from raster mask node.
         */
        std::tuple<std::size_t, value_type>
        loadRasterMask(unsigned int mask, std::istream &is);

        /** Load version 1.
         */
        std::tuple<std::size_t, value_type>
        loadV1(unsigned int mask, std::istream &is);

        std::tuple<std::size_t, QTree::value_type>
        loadChildren(unsigned int mask, std::istream &is);

        std::tuple<std::size_t, QTree::value_type>
        loadChildrenBw(unsigned int mask, std::istream &is);

        std::tuple<std::size_t, QTree::value_type>
        load(unsigned int mask, std::istream &is, std::uint8_t flags);

        std::tuple<std::size_t, QTree::value_type>
        loadBw(unsigned int mask, std::istream &is, std::uint8_t flags);

        /** Called from QTree::forEachNode */
        template <typename Op>
        void descend(unsigned int mask, unsigned int x, unsigned int y
                     , const Op &op, Filter filter) const;

        template <typename Op>
        void translate(const Op &op);

        template <typename FilterOp>
        void simplify(const FilterOp &filter);

        /** Nodes that satisfy condition (filter(value) == 0) are set to 0.
         */
        template <typename FilterOp>
        void filterOut(const FilterOp &filter);

        /** Merge nodes
         *  Uses type(filter) to determine node type.
         */
        template <typename FilterOp>
        void merge(const Node &other, const FilterOp &filter);

        /** Coarsen one level up.
         *  Uses type(op) to determine node type.
         *
         * \param size node size
         * \param filter determines node color (false->black, true->white)
         * \param force keeps "black" children as-is if false
         */
        template <typename FilterOp>
        void coarsen(unsigned int size, const FilterOp &filter
                     , bool force = true);

        /** Intersect nodes.
         */
        template <typename FilterOp>
        void intersect(const Node &other, const FilterOp &filter);

        /** Checks for intersection.
         */
        template <typename FilterOp>
        bool overlaps(const Node &other, const FilterOp &filter) const;

        template <typename FilterOp>
        bool find(const FilterOp &filter) const;

        enum Type { black, white, gray };
        template <typename FilterOp>
        Type type(const FilterOp &filter) const {
            if (children) { return Type::gray; }
            return filter(value) ? Type::white : Type::black;
        }

        /** Combines other node into this one.
         *  Can result in node split.
         */
        template <typename Combiner>
        void combine(// unsigned int size, unsigned int x, unsigned int y,
                     const Node &other, const Combiner &combiner);

        void shrink(unsigned int depth);

        const Node& find(unsigned int depth, unsigned int x
                         , unsigned int y) const;

        void force(value_type value);

        void swap(Node &other);

        template <typename Converter>
        void convert(Converter &converter) const;

        /** Returns value as boost optional.
         */
        opt_value_type optValue() const {
            if (children) { return boost::none; }
            return value;
        }

        /** Check valud by given matcher.
         */
        template <typename Matcher>
        bool match(const Matcher &matcher) const;

        template <typename Comparator>
        bool compare(const Node &other, const Comparator &comparator) const;
    };

    unsigned int order_;
    unsigned int size_;
    Node root_;
    std::size_t count_;
    value_type allSetFlags_;
};

/** Using std::function to hide implementation.
 */
void dump(const QTree &tree, const boost::filesystem::path &path
          , const std::function<bool(QTree::value_type)> &filter
          , double pixelSize = 1.0);

struct QTree::Node::Children {
    std::array<Node, 4> nodes; // ul, ur, ll, lr

    Children(QTree::value_type value = 0) { nodes.fill(value); }
    Children(const Children &other) : nodes(other.nodes) {}
    Children(Children&&) = default;
    Children& operator=(Children&&) = default;

    bool sameValue() const;
};

// inlines

template <typename Op>
inline void QTree::forEachNode(const Op &op, Filter filter) const
{
    root_.descend(size_, 0, 0, op, filter);
}

template <typename Op>
inline void QTree::forEach(const Op &op, Filter filter) const
{
    this->forEachNode([&](unsigned int x, unsigned int y, unsigned int size
                          , value_type value)
    {
        // rasterize node
        unsigned int ex(x + size);
        unsigned int ey(y + size);

        for (unsigned int j(y); j < ey; ++j) {
            for (unsigned int i(x); i < ex; ++i) {
                op(i, j, value);
            }
        }
    }, filter);
}

template <typename Op>
inline void QTree::Node::descend(unsigned int mask, unsigned int x
                                 , unsigned int y, const Op &op, Filter filter)
    const
{
    if (children) {
        // process children if allowed by depth
        if (auto nmask = (mask >> 1)) {
            const auto &n(children->nodes);
            n[0].descend(nmask, x, y, op, filter);
            n[1].descend(nmask, x + nmask, y, op, filter);
            n[2].descend(nmask, x, y + nmask, op, filter);
            n[3].descend(nmask, x + nmask, y + nmask, op, filter);
        }
        return;
    }

    switch (filter) {
    case Filter::black: if (value) { return; }; break;
    case Filter::white: if (!value) { return; }; break;
    default: break;
    }

    // call operation for node
    op(x, y, mask, value);
}

template <typename Op>
inline void QTree::forEachNode(unsigned int depth
                               , unsigned int x, unsigned int y
                               , const Op &op, Filter filter) const
{
    auto subdepth((depth < order_) ? (order_ - depth) : 1);
    root_.find(depth, x, y).descend(1 << subdepth, 0, 0, op, filter);
}

template <typename Op>
inline void QTree::forEach(unsigned int depth, unsigned int x, unsigned int y
                           , const Op &op, Filter filter) const
{
    this->forEachNode(depth, x, y
                      , [&](unsigned int x, unsigned int y, unsigned int size
                            , value_type value)
    {
        // rasterize node
        unsigned int ex(x + size);
        unsigned int ey(y + size);

        for (unsigned int j(y); j < ey; ++j) {
            for (unsigned int i(x); i < ex; ++i) {
                op(i, j, value);
            }
        }
    }, filter);
}

template <typename Op>
void QTree::translateEachNode(const Op &op)
{
    root_.translate(op);
    recount();
}

template <typename Op>
void QTree::Node::translate(const Op &op)
{
    if (children) {
        for (auto &node : children->nodes) {
            node.translate(op);
        }
        contract();
        return;
    }

    value = op(value);
}

template <typename FilterOp>
void QTree::simplify(const FilterOp &filter)
{
    root_.simplify(filter);
    recount();
}

template <typename FilterOp>
void QTree::Node::simplify(const FilterOp &filter)
{
    switch (type(filter)) {
    case Type::black: value = 0; break;
    case Type::white: value = 1; break;
    case Type::gray:
        children->nodes[0].simplify(filter);
        children->nodes[1].simplify(filter);
        children->nodes[2].simplify(filter);
        children->nodes[3].simplify(filter);
        contract();
        break;
    }
}

template <typename FilterOp>
inline void QTree::merge(const QTree &other, const FilterOp &filter)
{
    root_.merge(other.root_, filter);
    recount();
}

template <typename FilterOp>
void QTree::coarsen(const FilterOp &filter, bool resize)
{
    root_.coarsen(size_, filter, true);
    if (resize && order_) {
        --order_;
        size_ = 1 << order_;
    }
    recount();
}


template <typename FilterOp>
void QTree::conditionalCoarsen(const FilterOp &filter)
{
    root_.coarsen(size_, filter, false);
    recount();
}

template <typename FilterOp>
inline void QTree::intersect(const QTree &other, const FilterOp &filter)
{
    root_.intersect(other.root_, filter);
    recount();
}

template <typename FilterOp>
inline bool QTree::overlaps(const QTree &other, const FilterOp &filter) const
{
    return root_.overlaps(other.root_, filter);
}

template <typename FilterOp>
void QTree::Node::filterOut(const FilterOp &filter)
{
    if (children) {
        children->nodes[0].filterOut(filter);
        children->nodes[1].filterOut(filter);
        children->nodes[2].filterOut(filter);
        children->nodes[3].filterOut(filter);

        // contract if possible
        contract();
    }

    // BLACK -> zero
    if (!filter(value)) {
        value = 0;
    }
}

template <typename FilterOp>
void QTree::Node::merge(const Node &other, const FilterOp &filter)
{
    auto tt(type(filter));
    auto ot(other.type(filter));

    if ((tt == Type::white) || (ot == Type::black)) {
        // merge(WHITE, anything) = WHITE (keep)
        // merge(anything, BLACK) = anything (keep)
        return;
    }

    if (ot == Type::white) {
        // merge(anything, WHITE) = WHITE
        *this = other;
        return;
    }

    // OK, other is gray
    if (tt == Type::black) {
        // merge(BLACK, GRAY) = GRAY
        *this = other;
        filterOut(filter);
        return;
    }

    // merge(GRAY, GRAY) = go down
    children->nodes[0].merge(other.children->nodes[0], filter);
    children->nodes[1].merge(other.children->nodes[1], filter);
    children->nodes[2].merge(other.children->nodes[2], filter);
    children->nodes[3].merge(other.children->nodes[3], filter);

    // contract if possible
    contract();
}

template <typename FilterOp>
void QTree::Node::coarsen(unsigned int size, const FilterOp &filter
                          , bool force)
{
    if (size == 2) {
        if (!children) { return; }

        // 2-pixel non-leaf node

        if (force) {
            // forced coarsen: copy value of first white (or last black) node
            // into this one
            for (const auto &child : children->nodes) {
                value = child.value;
                if (filter(child.value)) { break; }
            }
            // always drop children
            children.reset();
            return;
        }

        // non-forced: keep all black nodes
        for (const auto &child : children->nodes) {
            if (filter(child.value)) {
                // only if filter matches
                value = child.value;
                // drop children
                children.reset();
                return;
            }
        }

        return;
    }

    // leaf -> leave
    if (!children) { return; }

    // non leaf -> descend
    size >>= 1;
    children->nodes[0].coarsen(size, filter, force);
    children->nodes[1].coarsen(size, filter, force);
    children->nodes[2].coarsen(size, filter, force);
    children->nodes[3].coarsen(size, filter, force);

    // contract if possible
    contract();
}

template <typename FilterOp>
void QTree::Node::intersect(const Node &other, const FilterOp &filter)
{
    auto tt(type(filter));
    auto ot(other.type(filter));

    if (tt == Type::black) {
        // intersect(BLACK, anything) = BLACK
        return;
    }

    if (tt == Type::white) {
        if (ot == Type::black) {
            // intersect(WHITE, BLACK) = BLACK
            *this = other;
            return;
        } else if (ot == Type::white) {
            // intersect(WHITE, WHITE) = WHITE
            return;
        }

        // intersect(WHITE, GRAY) = GRAY
        *this = other;
        return;
    } else {
        // this is a gray node
        if (ot == Type::black) {
            // intersect(GRAY, BLACK) = BLACK
            *this = other;
            return;
        } else if (ot == Type::white) {
            // intersect(GRAY, WHITE) = GRAY
            return;
        }
    }

    // intersect(GRAY, GRAY);

    // go down
    children->nodes[0].intersect(other.children->nodes[0], filter);
    children->nodes[1].intersect(other.children->nodes[1], filter);
    children->nodes[2].intersect(other.children->nodes[2], filter);
    children->nodes[3].intersect(other.children->nodes[3], filter);

    // contract if possible
    contract();
}

template <typename FilterOp>
bool QTree::Node::overlaps(const Node &other, const FilterOp &filter) const
{
    auto tt(type(filter));
    auto ot(other.type(filter));

    if (tt == Type::black) {
        // intersect(BLACK, anything) = BLACK
        return false;
    }

    if (tt == Type::white) {
        if (ot == Type::black) {
            // intersect(WHITE, BLACK) = BLACK
            return false;
        } else if (ot == Type::white) {
            // intersect(WHITE, WHITE) = WHITE
            return true;
        }

        // intersect(WHITE, GRAY) = value of GRAY
        return other.find(filter);
    } else {
        // this is a gray node
        if (ot == Type::black) {
            // intersect(GRAY, BLACK) = BLACK
            return false;
        } else if (ot == Type::white) {
            // intersect(GRAY, WHITE) = value of GRAY
            return find(filter);
        }
    }

    // intersect(GRAY, GRAY);

    // go down
    return (children->nodes[0].overlaps(other.children->nodes[0], filter)
            || children->nodes[1].overlaps(other.children->nodes[1], filter)
            || children->nodes[2].overlaps(other.children->nodes[2], filter)
            || children->nodes[3].overlaps(other.children->nodes[3], filter));
}

template <typename FilterOp>
bool QTree::Node::find(const FilterOp &filter) const
{
    if (children) {
        return (children->nodes[0].find(filter)
                || children->nodes[1].find(filter)
                || children->nodes[2].find(filter)
                || children->nodes[3].find(filter));
    }

    return filter(value);
}

template <typename Combiner>
void QTree::combine(const QTree &other, const Combiner &combiner)
{
    root_.combine(/*size_, 0, 0,*/ other.root_, combiner);
    recount();
}

template <typename Combiner>
void QTree::Node::combine(//unsigned int size, unsigned int x, unsigned int y,
                          const Node &other, const Combiner &combiner)
{
    // auto hsize(size >> 1);
    if (children) {
        // inner node
        if (other.children) {
            // also inner node: descend in both trees
            children->nodes[0].combine(//hsize, x, y,
                                       other.children->nodes[0], combiner);
            children->nodes[1].combine(//hsize, x + hsize, y,
                                       other.children->nodes[1], combiner);
            children->nodes[2].combine(//hsize, x, y + hsize,
                                       other.children->nodes[2], combiner);
            children->nodes[3].combine(//hsize, x + hsize, y + hsize,
                                       other.children->nodes[3], combiner);

            contract();
            return;
        }

        // leaf node: virtually split and descend
        children->nodes[0].combine(//hsize, x, y,
                                   other, combiner);
        children->nodes[1].combine(//hsize, x + hsize, y,
                                   other, combiner);
        children->nodes[2].combine(//hsize, x, y + hsize,
                                   other, combiner);
        children->nodes[3].combine(//hsize, x + hsize, y + hsize,
                                   other, combiner);

        contract();
        return;
    }


    // leaf node
    if (other.children) {
        // inner node: we need to physically split this node
        children.reset(new Children(value));

        // and descend
        children->nodes[0].combine(//hsize, x, y,
                                   other.children->nodes[0], combiner);
        children->nodes[1].combine(//hsize, x + hsize, y,
                                   other.children->nodes[1], combiner);
        children->nodes[2].combine(//hsize, x, y + hsize,
                                   other.children->nodes[2], combiner);
        children->nodes[3].combine(//hsize, x + hsize, y + hsize,
                                   other.children->nodes[3], combiner);

        contract();
        return;
    }


    // leafs finally meet together
    value = combiner(/*size, x, y,*/ value, other.value);

    // nothing to contract here since we are at a leaf :)
}

template <typename Op>
void QTree::update(unsigned int x1, unsigned int y1
                   , unsigned int x2, unsigned int y2, Op op)
{
    // NB: left/top boundary not checked since we use unsigned numbers

    // check for block completely outside the pane
    if ((x1 >= size_) || (y1 >= size_)) { return; }

    // clip to pane
    if (x2 >= size_) { x2 = size_ - 1; }
    if (y2 >= size_) { y2 = size_ - 1; }

    // and go down
    count_ += root_.update(size_, 0, 0, x1, y1, x2, y2, op);
}

template <typename Op>
long QTree::Node::update(unsigned int size, unsigned int x, unsigned int y
                         , unsigned int x1, unsigned int y1
                         , unsigned int x2, unsigned int y2, Op op)
{
    if ((x2 < x) || (x1 >= (x + size))
        || (y2 < y) || (y1 >= (y + size)))
    {
        // outside of given range -> nothing to do
        return 0;
    }

    if (!children) {
        if ((x1 <= x) && (x2 >= (x + size - 1))
            && (y1 <= y) && (y2 >= (y + size - 1)))
        {
            // found leaf node inside range: update and leave
            auto oldValue(this->value);
            auto newValue(op(this->value));
            if (oldValue == newValue) {
                // nothing to change
                return 0;
            }

            // apply change
            this->value = newValue;
            if ((oldValue && newValue) || (!oldValue && !newValue)) {
                return 0;
            }
            if (oldValue && !newValue) { return -(size * size); }
            return (size * size);
        }

        // not at the leaf node -> we have to split this node and descend
        children.reset(new Children(this->value));
    }

    // go down to all four children
    size >>= 1;
    long res(0);
    res += children->nodes[0].update
        (size, x, y, x1, y1, x2, y2, op);
    res += children->nodes[1].update
        (size, x + size, y, x1, y1, x2, y2, op);
    res += children->nodes[2].update
        (size, x, y + size, x1, y1, x2, y2, op);
    res += children->nodes[3].update
        (size, x + size, y + size, x1, y1, x2, y2, op);

    // contract nodes of same value
    contract();
    return res;
}

inline void QTree::swap(QTree &other) {
    std::swap(order_, other.order_);
    std::swap(size_, other.size_);
    root_.swap(other.root_);
    std::swap(count_, other.count_);
    std::swap(allSetFlags_, other.allSetFlags_);
}

inline void QTree::Node::swap(Node &other)
{
    std::swap(value, other.value);
    std::swap(children, other.children);
}

template <typename Converter>
void QTree::convert(Converter &converter) const
{
    converter.root(root_.optValue());
    if (root_.children) {
        // descend
        root_.convert(converter);
    }
}

template <typename Converter>
void QTree::Node::convert(Converter &converter) const
{
    const auto &nodes(children->nodes);

    // pass value of all 4 child nodes to children callback and colled whatever
    // it retuns
    const auto any(converter.children
                   (nodes[0].optValue(), nodes[1].optValue()
                    , nodes[2].optValue(), nodes[3].optValue()));

    // descend to all 4 nodes
    for (int i(0); i < 4; ++i) {
        const auto &node(nodes[i]);
        if (!node.children) { continue; }

        converter.enter(any, i);
        node.convert(converter);
        converter.leave(any, i);
    }
}

template <typename Matcher>
bool QTree::matchAll(const Matcher &matcher) const
{
    return root_.match(matcher);
}

template <typename Matcher>
bool QTree::Node::match(const Matcher &matcher) const
{
    if (children) {
        return (children->nodes[0].match(matcher)
                && children->nodes[1].match(matcher)
                && children->nodes[2].match(matcher)
                && children->nodes[3].match(matcher));
    }

    return matcher(value);
}

template <typename Comparator>
bool QTree::compare(const QTree &other, const Comparator &comparator) const
{
    return root_.compare(other.root_, comparator);
}

template <typename Comparator>
bool QTree::Node::compare(const Node &other, const Comparator &comparator)
    const
{
    if (children) {
        // internal + ?
        const auto &n(children->nodes);

        if (other.children) {
            // internal + internal: full recurse, child with child
            const auto &cn(other.children->nodes);
            return (n[0].compare(cn[0], comparator)
                    && n[1].compare(cn[1], comparator)
                    && n[2].compare(cn[2], comparator)
                    && n[3].compare(cn[3], comparator));
        }

        // internal + leaf: recurse into children and pass leaf
        return (n[0].compare(other, comparator)
                && n[1].compare(other, comparator)
                && n[2].compare(other, comparator)
                && n[3].compare(other, comparator));
    }

    // leaf + ?
    if (other.children) {
        // leaf + internal: recurse into self and pass children
        const auto &cn(other.children->nodes);
        return (compare(cn[0], comparator)
                && compare(cn[1], comparator)
                && compare(cn[2], comparator)
                && compare(cn[3], comparator));
    }

    // leaf + leaf, just compare
    return comparator(value, other.value);
}

} } // namespace vtslibs::vts

#endif // vtslibs_vts_qtree_hpp_included_
