#include <boost/gil/gil_all.hpp>

#include "dbglog/dbglog.hpp"

#include "./qtree.hpp"

namespace vadstena { namespace vts {

template <bool color, typename View>
void rasterize(const QTree &tree, View &view)
{
    // const auto size(tree.size());
    // const math::Size2 vsize(view.width(), view.height());

    // TODO: check sizes

    tree.forEachNode([&](unsigned int x, unsigned int y, unsigned int size
                         , QTree::value_type value)
    {
        boost::gil::fill_pixels
            (boost::gil::subimage_view(view, x, y, size, size)
             , color ? value : 0xff);
    }, QTree::Filter::white);
}

template <typename View>
void rasterize(const QTree &tree, View &view)
{
    return rasterize<true, View>(tree, view);
}

} } // namespace vadstena::vts
