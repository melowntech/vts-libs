#include <algorithm>

#include <opencv2/highgui/highgui.hpp>

#include "dbglog/dbglog.hpp"

#include "utility/binaryio.hpp"

#include "../../storage/error.hpp"

#include "./navtile.hpp"

namespace vadstena { namespace vts { namespace opencv {

void NavTile::serialize(std::ostream &os) const
{
    // convert data to image
    const auto ts(NavTile::size());
    cv::Mat image(ts.height, ts.width, CV_8UC1);

    const auto hr(heightRange());

    auto size(hr.size());
    std::transform(data_.begin<double>(), data_.end<double>()
                   , image.begin<std::uint8_t>()
                   , [&](double value) -> std::uint8_t
    {
        return std::uint8_t(std::round((255 * (value - hr.min)) / size));
    });

    // serialize image
    using utility::binaryio::write;
    std::vector<unsigned char> buf;
    cv::imencode(".jpg", image, buf
                 , { cv::IMWRITE_JPEG_QUALITY, 92 });

    write(os, buf.data(), buf.size());
}

void NavTile::deserialize(const HeightRange &heightRange
                          , std::istream &is
                          , const boost::filesystem::path &path)
{
    using utility::binaryio::read;
    auto size(is.seekg(0, std::ios_base::end).tellg());
    is.seekg(0);
    std::vector<unsigned char> buf;
    buf.resize(size);
    read(is, buf.data(), buf.size());

    auto image(cv::imdecode(buf, CV_LOAD_IMAGE_GRAYSCALE));
    if (!image.data) {
        LOGTHROW(err1, storage::FormatError)
            << "Cannot deserialize navtile from file " << path << ".";
    }

    // check sizes
    auto ts(NavTile::size());
    if ((image.rows != ts.height) || (image.cols != ts.width)) {
        LOGTHROW(err1, storage::FormatError)
            << "Navtile in file " << path
            << "has different dimensions than " << ts << ".";
    }

    data_.create(ts.height, ts.width, CV_64FC1);
    std::transform(image.begin<std::uint8_t>(), image.end<std::uint8_t>()
                   , data_.begin<double>()
                   , [&](std::uint8_t value) -> double
    {
        return ((double(heightRange.min) * (255.0 - value)
                 + double(heightRange.max) * value)
                / 255.0);
    });
}

NavTile::HeightRange NavTile::heightRange() const
{
    // TODO: filter-out masked-out values
    // get min/max
    storage::Range<double> range;
    cv::minMaxLoc(data_, &range.min, &range.max);
    return HeightRange(std::int16_t(std::floor(range.min))
                       , std::int16_t(std::ceil(range.max)));
}

void NavTile::data(const Data &data)
{
    // check size
    auto ts(NavTile::size());
    if ((data.rows != ts.height) || (data.cols != ts.width)) {
        LOGTHROW(err1, storage::FormatError)
            << "Passed navigational data have different dimensions than "
            << ts << ".";
    }

    // check type
    if (data.type() != CV_64FC1) {
        LOGTHROW(err1, storage::FormatError)
            << "Passed navigational data is not "
            "a single channel double matrix.";
    }

    // clone
    data_ = data.clone();
}

NavTile::Data NavTile::createData(boost::optional<double> value)
{
    auto ts(NavTile::size());
    Data data(ts.height, ts.width, CV_64FC1);
    if (value) { data = cv::Scalar(*value); }
    return data;
}

} } } // namespace vadstena::vts::opencv
