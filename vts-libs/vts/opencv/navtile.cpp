/**
 * Copyright (c) 2017 Melown Technologies SE
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * *  Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#include <algorithm>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/photo/photo.hpp>

#include "dbglog/dbglog.hpp"

#include "utility/binaryio.hpp"

#include "imgproc/binterpolate.hpp"
#include "imgproc/fillrect.hpp"

#include "../../storage/error.hpp"

#include "navtile.hpp"

namespace vtslibs { namespace vts { namespace opencv {

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
    storage::Range<double> hr(heightRange());

    double size(hr.size());
    if (size < 1e-6) {
        // flat -> single value
        return writeImage
            (cv::Mat(ts.height, ts.width, CV_8UC1, cv::Scalar(128)), os);
    }

    // non-flat
    if (coverageMask().full()) {
        // fully covered
        cv::Mat image(ts.height, ts.width, CV_8UC1);
        std::transform(data_.begin<DataType>(), data_.end<DataType>()
                       , image.begin<std::uint8_t>()
                       , [&](DataType value) -> std::uint8_t
        {
            return std::uint8_t
                (std::round((255 * (clamp(hr, value) - hr.min)) / size));
        });
        return writeImage(image, os);
    }

    // partially covered: process only valid part and then inpaint invalid part
    // to get something smooth
    int margin(2);
    math::Size2 mts(ts.width + 2 * margin, ts.height + 2 * margin);
    cv::Mat image(mts.height, mts.width, CV_8UC1, cv::Scalar(128));
    cv::Mat mask(mts.height, mts.width, CV_8UC1, cv::Scalar(255));

    // render valid pixels and mask them out in the inpaint mask
    coverageMask().forEach([&](int x, int y, bool)
    {
        auto value(data_.at<DataType>(y, x));
        image.at<std::uint8_t>(y + margin, x + margin)
            = std::uint8_t
            (std::round((255 * (clamp(hr, value) - hr.min)) / size));
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

    data_.create(ts.height, ts.width, CvDataType);
    std::transform(image.begin<std::uint8_t>(), image.end<std::uint8_t>()
                   , data_.begin<DataType>()
                   , [&](std::uint8_t value) -> DataType
    {
        return ((DataType(heightRange.min) * (255.f - value)
                 + DataType(heightRange.max) * value)
                / 255.f);
    });
}

void NavTile::heightRange(const HeightRange &heightRange)
{
    heightRange_ = heightRange;
}

NavTile::HeightRange NavTile::heightRange() const
{
    // override
    if (heightRange_) { return *heightRange_; }

    // get min/max from valid pixels
    storage::Range<DataType> range(0, 0);
    if (!coverageMask().empty()) {
        range = { std::numeric_limits<DataType>::max()
                  , std::numeric_limits<DataType>::lowest() };
        coverageMask().forEach([&](int x, int y, bool)
        {
            auto value(data_.at<DataType>(y, x));
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

    data.assignTo(data_, CvDataType);
}

void NavTile::data(const Data &data, const Data &mask)
{
    // set data
    this->data(data);

    // fill in mask
    auto &cm(coverageMask());
    cm.reset();
    for (int j(0); j < mask.rows; ++j) {
        for (int i(0); i < mask.cols; ++i) {
            if (!mask.at<std::uint8_t>(j, i)) {
                cm.set(i, j, false);
            }
        }
    }
}

NavTile::Data NavTile::createData(boost::optional<DataType> value)
{
    auto ts(NavTile::size());
    Data data(ts.height, ts.width, CvDataType);
    if (value) { data = cv::Scalar(*value); }
    return data;
}

cv::Mat renderCoverage(const NavTile &navtile)
{
    const auto nts(NavTile::size());
    cv::Mat coverage(nts.height, nts.width, CV_8U, cv::Scalar(0));

    const cv::Scalar white(255);
    navtile.coverageMask()
        .forEachQuad([&](uint xstart, uint ystart, uint xsize
                         , uint ysize, bool)
    {
        imgproc::fillRectangle(coverage, cv::Rect(xstart, ystart, xsize, ysize)
                               , white);
    }, NavTile::CoverageMask::Filter::white);

    return coverage;
}

NavTile::DataType NavTile::sample(const math::Point2 &p) const
{
    DataType height(0.0);
    imgproc::bilinearInterpolate(data_.ptr<DataType>(), data_.step1()
                                 , data_.cols, data_.rows
                                 , 1, p, &height);
    return height;
}

} } } // namespace vtslibs::vts::opencv
