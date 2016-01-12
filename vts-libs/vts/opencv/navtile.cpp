#include <algorithm>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/photo/photo.hpp>

#include "dbglog/dbglog.hpp"

#include "utility/binaryio.hpp"

#include "../../storage/error.hpp"

#include "./navtile.hpp"

namespace vadstena { namespace vts { namespace opencv {

namespace {

multifile::Table writeImage(const cv::Mat &image, std::ostream &os)
{
    // write
    multifile::Table table;
    auto pos(os.tellp());

    // serialize image
    using utility::binaryio::write;
    std::vector<unsigned char> buf;
    // TODO: configurable quality?
    cv::imencode(".jpg", image, buf, { cv::IMWRITE_JPEG_QUALITY, 92 });

    write(os, buf.data(), buf.size());
    pos = table.add(pos, buf.size());

    return table;
}

} // namespace

multifile::Table NavTile::serialize_impl(std::ostream &os) const
{
    // convert data to image
    const auto ts(NavTile::size());
    const auto hr(heightRange());

    auto size(hr.size());
    if (size < 1e-6) {
        // flat -> single value
        return writeImage
            (cv::Mat(ts.height, ts.width, CV_8UC1, cv::Scalar(128)), os);
    }

    // non-flat
    if (coverageMask().full()) {
        // fully covered
        cv::Mat image(ts.height, ts.width, CV_8UC1);
        std::transform(data_.begin<float>(), data_.end<float>()
                       , image.begin<std::uint8_t>()
                       , [&](float value) -> std::uint8_t
        {
            return std::uint8_t
                (std::round((255 * (value - hr.min)) / size));
        });
        return writeImage(image, os);
    }

    // partially covered: process only valid part and then then inpaint
    // invalid part to get something smooth
    int margin(2);
    math::Size2 mts(ts.width + 2 * margin, ts.height + 2 * margin);
    cv::Mat image(mts.height, mts.width, CV_8UC1, cv::Scalar(128));
    cv::Mat mask(mts.height, mts.width, CV_8UC1, cv::Scalar(255));

    // render valid pixels and mask them out in the inpaint mask
    coverageMask().forEach([&](int x, int y, bool)
    {
        auto value(data_.at<float>(y, x));
        image.at<std::uint8_t>(y + margin, x + margin)
            = std::uint8_t(std::round((255 * (value - hr.min)) / size));
        mask.at<std::uint8_t>(y + margin, x + margin) = 0;
    }, CoverageMask::Filter::white);

    // inpaint
    cv::Mat tmp(mts.height, mts.width, CV_8UC1);
    cv::inpaint(image, mask, tmp, 3, cv::INPAINT_TELEA);

    // done
    return writeImage
        (cv::Mat(tmp, cv::Rect(margin, margin, ts.width, ts.height)), os);
}

void NavTile::deserialize_impl(const HeightRange &heightRange
                               , std::istream &is
                               , const boost::filesystem::path &path
                               , const multifile::Table &table)
{
    using utility::binaryio::read;

    const auto &entry(table[imageIndex()]);

    is.seekg(entry.start);
    std::vector<unsigned char> buf;
    buf.resize(entry.size);
    read(is, buf.data(), buf.size());

    auto image(cv::imdecode(buf, CV_LOAD_IMAGE_GRAYSCALE));
    if (!image.data) {
        LOGTHROW(err1, storage::BadFileFormat)
            << "Cannot decode navtile image from block(" << entry.start
            << ", " << entry.size << " in file " << path << ".";
    }

    // check sizes
    auto ts(NavTile::size());
    if ((image.rows != ts.height) || (image.cols != ts.width)) {
        LOGTHROW(err1, storage::FormatError)
            << "Navtile in file " << path
            << "has different dimensions than " << ts << ".";
    }

    data_.create(ts.height, ts.width, CV_32FC1);
    std::transform(image.begin<std::uint8_t>(), image.end<std::uint8_t>()
                   , data_.begin<float>()
                   , [&](std::uint8_t value) -> float
    {
        return ((float(heightRange.min) * (255.f - value)
                 + float(heightRange.max) * value)
                / 255.f);
    });
}

NavTile::HeightRange NavTile::heightRange() const
{
    // get min/max from valid pixels
    storage::Range<float> range(0, 0);
    if (!coverageMask().empty()) {
        range = { std::numeric_limits<float>::max()
                  , std::numeric_limits<float>::lowest() };
        coverageMask().forEach([&](int x, int y, bool)
        {
            auto value(data_.at<float>(y, x));
            range.min = std::min(range.min, value);
            range.max = std::max(range.max, value);
        }, CoverageMask::Filter::white);
    }
    return HeightRange(std::int16_t(std::floor(range.min))
                       , std::int16_t(std::ceil(range.max)));
}

void NavTile::data(const Data &data)
{
    // check size
    auto ts(NavTile::size());
    if ((data.rows != ts.height) || (data.cols != ts.width)) {
        LOGTHROW(err1, storage::FormatError)
            << "Passed navigation data have different dimensions than "
            << ts << ".";
    }

    // check type
    if (data.channels() != 1) {
        LOGTHROW(err1, storage::FormatError)
            << "Passed navigation data is not a single channel matrix.";
    }

    data.assignTo(data_, CV_32F);
}

NavTile::Data NavTile::createData(boost::optional<float> value)
{
    auto ts(NavTile::size());
    Data data(ts.height, ts.width, CV_32FC1);
    if (value) { data = cv::Scalar(*value); }
    return data;
}

} } } // namespace vadstena::vts::opencv
