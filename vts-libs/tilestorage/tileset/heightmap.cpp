#include <boost/format.hpp>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "imgproc/filtering-generic.hpp"

#include "../tileset-detail.hpp"

namespace vadstena { namespace tilestorage {

void TileSet::Detail::filterHeightmap(const TileIndex &changed, double hwin)
{
    LOG(info2)
        << "Filtering height map in LOD range "
        << changed.lodRange() << ".";
    for (auto lod : changed.lodRange()) {
        filterHeightmap(lod, changed, hwin);
    }
}

namespace {

void dumpMat(Lod lod, const std::string &name, const cv::Mat &mat)
{
    auto file(str(boost::format("changed-%s-%02d.png") % name % lod));
    cv::imwrite(file, mat);
}

cv::Mat createMask(Lod lod, const TileIndex &changed, int hwin, int margin)
{
    const auto &rmask(*changed.mask(lod));
    auto size(rmask.dims());
    size.width += (2 * margin);
    size.height += (2 * margin);

    // inside mask, clear
    cv::Mat inside(size.height, size.width, CV_8UC1);
    inside = cv::Scalar(0x00);

    // outside mask, fill with dilated outer border
    cv::Mat outside(size.height, size.width, CV_8UC1);
    // 1) full white
    outside = cv::Scalar(0xff);
    // 2) and clear inside
    {
        cv::Rect rect(cv::Point2i(margin + hwin, margin + hwin)
                      , cv::Size(size.width - 2 * (margin + hwin)
                                 , size.height - 2 * (margin + hwin)));
        if ((rect.width > 0) && (rect.height > 0)) {
            cv::rectangle(outside, rect
                          , cv::Scalar(0x00), CV_FILLED, 4);
        }
    }

    // render dilated valid quads in inside mask
    // render dilated invalid quads in outside mask
    const auto white(cv::Scalar(0xff));
    rmask.forEachQuad([&](uint xstart, uint ystart, uint xsize
                          , uint ysize, bool valid)
    {
        cv::Point2i start(xstart + margin - hwin
                          , ystart + margin - hwin);
        cv::Point2i end(xstart + xsize + margin + hwin - 1
                        , ystart + ysize + margin + hwin - 1);

        cv::rectangle((valid ? inside : outside)
                      , start, end, white, CV_FILLED, 4);
    }, RasterMask::Filter::both);

    dumpMat(lod, "inside", inside);
    dumpMat(lod, "outside", outside);

    // intersect inside and outside mask, place result into inside mask
    for (int j(0); j < size.height; ++j) {
        for (int i(0); i < size.width; ++i) {
            inside.at<unsigned char>(j, i)
                = 0xff * (inside.at<unsigned char>(j, i)
                          && outside.at<unsigned char>(j, i));
        }
    }

    dumpMat(lod, "mask", inside);
    return inside;
}

} // namespace

void TileSet::Detail::filterHeightmap(Lod lod, const TileIndex &changed
                                      , double hwin)
{
    LOG(info2) << "Filtering height map at LOD " << lod << ".";

    // half-window as an integer; add small epsilon to deal with numeric errors
    auto intHwin(int(std::ceil(hwin - 1e-5)));
    auto margin(intHwin);

    const auto getTileId([&](int i, int j)
    {
        return changed.tileId(lod, i - margin, j - margin);
    });

    auto flags(createMask(lod, changed, intHwin, margin));

    const math::CatmullRom2 filter(4 * hwin, 4 * hwin);

    auto value
        (imgproc::reconstruct(imgproc::MatReconstruction<unsigned char>(flags)
                              , filter, math::Point2i(flags.cols / 2
                                                      , flags.rows / 2)));
    LOG(info4) << "value: " << value;

    boost::gil::rgb8_image_t gimg(100, 100);
    auto gvalue
        (imgproc::reconstruct
         (imgproc::gilViewReconstruction(boost::gil::const_view(gimg))
          , filter, math::Point2i(gimg.width() / 2, gimg.height() / 2)));
    LOG(info4)
        << "gvalue: " << gvalue[0] << ", " << gvalue[1] << ", " << gvalue[3];

    // process raster
    for (int j(0); j < flags.rows; ++j) {
        for (int i(0); i < flags.cols; ++i) {
            // get flags
            if (!flags.at<unsigned char>(j, i)) { continue; }

            // regenerate tile
            auto tileId(getTileId(i, j));
            LOG(info2) << "Filtering tile [" << i << ", " << j << "] "
                       << tileId << ".";

        }
    }

    (void) filter;
}

} } // namespace vadstena::tilestorage
