#include <boost/gil/gil_all.hpp>

#include "dbglog/dbglog.hpp"

#include "./qtree.hpp"

namespace vtslibs { namespace vts {

template <typename View, typename Covert>
void rasterize(const QTree &tree, View &view, const Covert &convert);

template <typename View>
void rasterize(const QTree &tree, View &view);

template <typename View, typename Covert>
void rasterize(const QTree &tree
               , unsigned int depth, unsigned int x, unsigned int y
               , View &view, const Covert &convert);

template <typename View, typename Covert>
void rasterize(const QTree &tree
               , unsigned int depth, unsigned int x, unsigned int y
               , View &view);

// inlines

template <typename View, typename Covert>
inline void rasterize(const QTree &tree, View &view, const Covert &convert)
{
    if (tree.size() != math::Size2(view.width(), view.height())) {
        LOGTHROW(err1, std::runtime_error)
            << "Tree and view have incompatible sizes.";
    }

    tree.forEachNode([&](unsigned int x, unsigned int y, unsigned int size
                         , QTree::value_type value)
    {
        boost::gil::fill_pixels
            (boost::gil::subimage_view(view, x, y, size, size)
             , convert(value));
    }, QTree::Filter::white);
}

template <typename View>
inline void rasterize(const QTree &tree, View &view)
{
    return rasterize(tree, view
                     , [](QTree::value_type value)
                     {
                         return value;
                     });
}

template <typename View, typename Covert>
inline void rasterize(const QTree &tree
               , unsigned int depth, unsigned int x, unsigned int y
               , View &view, const Covert &convert)
{
    // if (tree.size() != math::Size2(view.width(), view.height())) {
    //     LOGTHROW(err1, std::runtime_error)
    //         << "Tree and view have incompatible sizes.";
    // }

    tree.forEachNode(depth, x, y
                     , [&](unsigned int x, unsigned int y, unsigned int size
                           , QTree::value_type value)
    {
        boost::gil::fill_pixels
            (boost::gil::subimage_view(view, x, y, size, size)
             , convert(value));
    }, QTree::Filter::white);
}

template <typename View, typename Covert>
inline void rasterize(const QTree &tree
                      , unsigned int depth, unsigned int x, unsigned int y
                      , View &view)
{
    return rasterize(tree, depth, x, y, view
                     , [](QTree::value_type value)
                     {
                         return value;
                     });
}

} } // namespace vtslibs::vts
