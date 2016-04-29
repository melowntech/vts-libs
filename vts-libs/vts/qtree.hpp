#ifndef vadstena_libs_vts_qtree_hpp_included_
#define vadstena_libs_vts_qtree_hpp_included_

#include <cstdint>
#include <array>
#include <iostream>
#include <iostream>
#include <functional>

#include <boost/filesystem/path.hpp>

#include "dbglog/dbglog.hpp"

#include "math/geometry_core.hpp"

namespace vadstena { namespace vts {

class QTree {
public:
    typedef std::uint32_t value_type;

    QTree(unsigned int order = 0, value_type value = 0);

    QTree(const QTree &other);
    QTree& operator=(const QTree &other);

    QTree(QTree&&) = default;
    QTree& operator=(QTree&&) = default;

    value_type get(unsigned int x, unsigned int y) const;

    void set(unsigned int x, unsigned int y, value_type value);

    /** Set given range.
     */
    void set(unsigned int x1, unsigned int y1
             , unsigned int x2, unsigned int y2, value_type value);

    void reset(value_type value);

    void save(std::ostream &os) const;
    void load(std::istream &is, const boost::filesystem::path &path);

    void recreate(unsigned int order = 0, value_type value = 0);

    unsigned int order() const { return order_; }

    /** Merge nodes.
     */
    template <typename FilterOp>
    void merge(const QTree &other, const FilterOp &filter);

    /** Coarsen one level up.
     */
    template <typename FilterOp>
    void coarsen(const FilterOp &filter);

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

    math::Size2 size() const { return math::Size2(size_, size_); }

    enum class Filter {
        black, white, both
    };

    /** Runs op(x, y, xsize, value) for each node based on filter.
     */
    template <typename Op>
    void forEachNode(const Op &op, Filter filter = Filter::both) const;

    /** Runs op(x, y, value) for each element base on filter.
     *  Uses forEachNode and rasterizes node internally.
     */
    template <typename Op>
    void forEach(const Op &op, Filter filter = Filter::both) const;

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

private:
    /** Re-calculates number of non-zero elements.
     */
    void recount();

    struct Node {
        // NB: value MSB is used as value marker in serialized data!
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

        void contract();

        void save(std::ostream &os) const;
        std::size_t load(unsigned int mask, std::istream &is);

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
         */
        template <typename FilterOp>
        void coarsen(unsigned int size, const FilterOp &filter);

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
    };

    unsigned int order_;
    unsigned int size_;
    Node root_;
    std::size_t count_;
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
        uint ex(x + size);
        uint ey(y + size);

        for (uint j(y); j < ey; ++j) {
            for (uint i(x); i < ex; ++i) {
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
        // process children
        auto nmask(mask >> 1);
        const auto &n(children->nodes);
        n[0].descend(nmask, x, y, op, filter);
        n[1].descend(nmask, x + nmask, y, op, filter);
        n[2].descend(nmask, x, y + nmask, op, filter);
        n[3].descend(nmask, x + nmask, y + nmask, op, filter);
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
void QTree::coarsen(const FilterOp &filter)
{
    root_.coarsen(size_, filter);
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
void QTree::Node::coarsen(unsigned int size, const FilterOp &filter)
{
    if (size == 2) {
        if (!children) { return; }

        // 2-pixel non-leaf node -> copy value of first while (or last black)
        // node into this one
        for (const auto &child : children->nodes) {
            value = child.value;
            if (filter(child.value)) { break; }
        }

        // drop children
        children.reset();
        return;
    }

    // leaf -> leave
    if (!children) { return; }

    // non leaf -> descend
    size >>= 1;
    children->nodes[0].coarsen(size, filter);
    children->nodes[1].coarsen(size, filter);
    children->nodes[2].coarsen(size, filter);
    children->nodes[3].coarsen(size, filter);

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
        // inner node: we to physically split this node
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

} } // namespace vadstena::vts

#endif // vadstena_libs_vts_qtree_hpp_included_
