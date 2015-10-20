#ifndef vadstena_libs_vts_qtree_hpp_included_
#define vadstena_libs_vts_qtree_hpp_included_

#include <cstdint>
#include <array>
#include <iostream>

#include <boost/filesystem/path.hpp>

namespace vadstena { namespace vts {

class QTree {
public:
    typedef std::uint8_t value_type;

    QTree(unsigned int order = 0, value_type value = 0);

    QTree(const QTree &other);
    QTree& operator=(const QTree &other);

    QTree(QTree&&) = default;
    QTree& operator=(QTree&&) = default;

    value_type get(unsigned int x, unsigned int y) const;

    void set(unsigned int x, unsigned int y, value_type value);

    void reset(value_type value);

    void save(std::ostream &os) const;
    void load(std::istream &is, const boost::filesystem::path &path);

    void recreate(unsigned int order = 0, value_type value = 0);

    unsigned int order() const { return order_; }

    /** Merge nodes.
     */
    template <typename FilterOp>
    void merge(const QTree &other, const FilterOp &filter);

    /** Intersect nodes.
     */
    template <typename FilterOp>
    void intersect(const QTree &other, const FilterOp &filter);

    /** Coarsen one level up.
     */
    template <typename FilterOp>
    void coarsen(const FilterOp &filter);

    /** Returns number of non-zero elements.
     */
    std::size_t count() const { return count_; }

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

        void contract();

        void save(std::ostream &os) const;
        std::size_t load(unsigned int mask, std::istream &is);

        /** Called from QTree::forEachNode */
        template <typename Op>
        void descend(unsigned int mask, unsigned int x, unsigned int y
                     , const Op &op, Filter filter) const;

        template <typename Op>
        void translate(const Op &op);

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

        enum Type { black, white, gray };
        template <typename FilterOp>
        Type type(const FilterOp &filter) const {
            if (children) { return Type::gray; }
            return filter(value) ? Type::white : Type::black;
        }
    };

    unsigned int order_;
    unsigned int size_;
    Node root_;
    std::size_t count_;
};

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
            node.translate();
        }
        contract();
        return;
    }

    value = Op(value);
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
        // mark as present and drfilter any children
        value = 1;
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
            value = 0;
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
            children.reset();
            value = 0;
            return;
        } else if (ot == Type::white) {
            // intersect(GRAY, WHITE) = GRAY
            return;
        }
    }

    // intersect(GRAY, GRAY);

    // go down
    children->nodes[0].intersect(other, filter);
    children->nodes[1].intersect(other, filter);
    children->nodes[2].intersect(other, filter);
    children->nodes[3].intersect(other, filter);

    // contract if possible
    contract();
}

} } // namespace vadstena::vts

#endif // vadstena_libs_vts_qtree_hpp_included_
