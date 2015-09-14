#include <opencv2/highgui/highgui.hpp>

#include "utility/binaryio.hpp"

#include "./atlas.hpp"

namespace vadstena { namespace vts { namespace opencv {

Atlas::Table Atlas::serialize_impl(std::ostream &os) const
{
    Table table;
    std::size_t pos(0);

    for (const auto &image : images_) {
        using utility::binaryio::write;
        std::vector<unsigned char> buf;
        cv::imencode(".jpg", image, buf
                     , { cv::IMWRITE_JPEG_QUALITY, quality_ });

        write(os, buf.data(), buf.size());
        table.emplace_back(pos, buf.size());
        pos += buf.size();
    }

    return table;
}

void Atlas::deserialize_impl(std::istream &is, const Table &table)
{
    Images images;
    for (const auto &entry : table) {
        using utility::binaryio::read;

        is.seekg(entry.start);
        std::vector<unsigned char> buf;
        buf.resize(entry.size);
        read(is, buf.data(), buf.size());

        auto image(cv::imdecode(buf, CV_LOAD_IMAGE_COLOR));
        images.push_back(image);
    }
    images_.swap(images);
}

void Atlas::set(std::size_t index, const Image &image)
{
    if (images_.size() <= index) {
        images_.resize(index + 1);
    }
    images_[index] = image;
}

std::size_t Atlas::area(std::size_t index) const
{
    if (index >= images_.size()) { return 0; }
    const auto &image(images_[index]);
    return image.cols * image.rows;
}

} } } // namespace vadstena::vts::opencv
