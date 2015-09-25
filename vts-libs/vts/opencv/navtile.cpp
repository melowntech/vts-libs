#include <algorithm>

#include <opencv2/highgui/highgui.hpp>

#include "dbglog/dbglog.hpp"

#include "utility/binaryio.hpp"

#include "../../storage/error.hpp"

#include "./navtile.hpp"

namespace vadstena { namespace vts { namespace opencv {

multifile::Table NavTile::serialize_impl(std::ostream &os) const
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

    // write
    multifile::Table table;
    auto pos(os.tellp());

    // serialize image
    using utility::binaryio::write;
    std::vector<unsigned char> buf;
    // TODO: configurable quality?
    cv::imencode(".jpg", image, buf
                 , { cv::IMWRITE_JPEG_QUALITY, 92 });

    write(os, buf.data(), buf.size());
    pos = table.add(pos, buf.size());

    return table;
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
