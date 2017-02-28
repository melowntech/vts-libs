#include <opencv2/highgui/highgui.hpp>

#include "dbglog/dbglog.hpp"
#include "utility/binaryio.hpp"

#include "./qtree.hpp"

namespace fs = boost::filesystem;
namespace bin = utility::binaryio;

namespace vtslibs { namespace vts {

void dump(const QTree &tree, const fs::path &path
          , const std::function<bool(QTree::value_type)> &filter
          , double pixelSize)
{
    const auto size(tree.size());
    cv::Mat m(long(std::ceil(pixelSize * size.height))
              , long(std::ceil(pixelSize * size.width))
              , CV_8UC1);
    m = cv::Scalar(0);
    auto white(cv::Scalar(0xff));

    uint fracAdj(std::round(pixelSize) - pixelSize != 0.0 ? 1 : 0);

    tree.forEachNode([&](unsigned int x, unsigned int y, unsigned int size
                         , QTree::value_type value)
    {
        if (!(filter(value))) { return; }

        cv::Point2i start(int(std::floor(pixelSize * x))
                          , int(std::floor(pixelSize * y)));
        cv::Point2i end
            (int(std::ceil(pixelSize * (x + size - fracAdj )))
             , int(std::ceil(pixelSize * (y + size - fracAdj))));

        cv::rectangle(m, start, end, white, CV_FILLED, 4);
    }, QTree::Filter::white);

    imwrite(path.string(), m);
}

} } // namespace vtslibs::vts
