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

    void merge(const QTree &other, bool checkDimensions = true);

    /** Returns number of non-zero elements.
     */
    std::size_t count() const { return count_; }

    enum class Filter {
        black, white, both
    };

    /** Runs op(x, y, xsize, value) for each quad based on filter.
     */
    template <typename Op>
    void forEachQuad(const Op &op, Filter filter = Filter::both) const;

    /** Runs op(x, y, value) for each element base on filter.
     *  Uses forEachQuad and rasterizes quad internally.
     */
    template <typename Op>
    void forEach(const Op &op, Filter filter = Filter::both) const;

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
        std::size_t load(std::istream &is);

        /** Called from QTree::forEachQuad */
        template <typename Op>
        void descend(unsigned int mask, unsigned int x, unsigned int y
                     , const Op &op, Filter filter) const;
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
inline void QTree::forEachQuad(const Op &op, Filter filter) const
{
    root_.descend(size_, 0, 0, op, filter);
}

template <typename Op>
inline void QTree::forEach(const Op &op, Filter filter) const
{
    this->forEachQuad([&](unsigned int x, unsigned int y, unsigned int size
                          , value_type value)
    {
        // rasterize quad
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

} } // namespace vadstena::vts

#endif // vadstena_libs_vts_qtree_hpp_included_
