#include <opencv2/highgui/highgui.hpp>

#include "dbglog/dbglog.hpp"

#include "utility/binaryio.hpp"

#include "imgproc/jpeg.hpp"

#include "../../storage/error.hpp"

#include "./atlas.hpp"

namespace vadstena { namespace vts { namespace opencv {

namespace {

std::vector<unsigned char> mat2jpeg(const cv::Mat &mat, int quality)
{
    std::vector<unsigned char> buf;
    cv::imencode(".jpg", mat, buf
                 , { cv::IMWRITE_JPEG_QUALITY, quality });
    return buf;
}

cv::Mat jpeg2mat(const std::vector<unsigned char> &buf
                 , const multifile::Table::Entry *entry = nullptr
                 , const boost::filesystem::path *path = nullptr)
{
    auto image(cv::imdecode(buf, CV_LOAD_IMAGE_COLOR));
    if (!image.data) {
        if (entry) {
            LOGTHROW(err1, storage::BadFileFormat)
                << "Cannot decode image from block(" << entry->start
                << ", " << entry->size << " in file " << *path << ".";
        }
        LOGTHROW(err1, storage::BadFileFormat)
            << "Cannot decode image from memory buffer " << buf.size()
            << " bytes long.";
    }
    return image;
}

} // namespace

multifile::Table Atlas::serialize_impl(std::ostream &os) const
{
    multifile::Table table;
    auto pos(os.tellp());

    for (const auto &image : images_) {
        using utility::binaryio::write;
        auto buf(mat2jpeg(image, quality_));
        write(os, buf.data(), buf.size());
        pos = table.add(pos, buf.size());
    }

    return table;
}

void Atlas::deserialize_impl(std::istream &is
                             , const boost::filesystem::path &path
                             , const multifile::Table &table)
{
    Images images;
    for (const auto &entry : table) {
        using utility::binaryio::read;

        is.seekg(entry.start);
        std::vector<unsigned char> buf;
        buf.resize(entry.size);
        read(is, buf.data(), buf.size());

        images.push_back(jpeg2mat(buf, &entry, &path));
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

math::Size2 Atlas::imageSize_impl(std::size_t index) const
{
    if (index >= images_.size()) { return {}; }
    const auto &image(images_[index]);
    return math::Size2(image.cols, image.rows);
}

HybridAtlas::HybridAtlas(std::size_t count, const RawAtlas &rawAtlas
                         , int quality)
    : quality_(quality)
{
    for (std::size_t i(0), e(std::min(rawAtlas.size(), count)); i != e; ++i) {
        entries_.emplace_back(rawAtlas.get(i));
    }
}

// Hybrid Atlas: used to build atlas from raw jpeg data (i.e. copy from another
// atlas) or from color images stored in OpenCV matrices
void HybridAtlas::add(const Image &image)
{
    entries_.emplace_back(image);
}

void HybridAtlas::add(const Raw &raw)
{
    entries_.emplace_back(raw);
}

HybridAtlas::Image HybridAtlas::imageFromRaw(const Raw &raw)
{
    return jpeg2mat(raw);
}

HybridAtlas::Raw HybridAtlas::rawFromImage(const Image &image, int quality)
{
    return mat2jpeg(image, quality);
}

multifile::Table HybridAtlas::serialize_impl(std::ostream &os) const
{
    multifile::Table table;
    auto pos(os.tellp());

    for (const auto &entry : entries_) {
        using utility::binaryio::write;
        if (entry.image.data) {
            auto buf(mat2jpeg(entry.image, quality_));
            write(os, buf.data(), buf.size());
            pos = table.add(pos, buf.size());
        } else {
            write(os, entry.raw.data(), entry.raw.size());
            pos = table.add(pos, entry.raw.size());
        }
    }

    return table;
}

void HybridAtlas::deserialize_impl(std::istream&
                                   , const boost::filesystem::path&
                                   , const multifile::Table&)
{
    LOGTHROW(err4, std::runtime_error)
        << "This atlas is serialize-only.";
}

math::Size2 HybridAtlas::imageSize_impl(std::size_t index) const
{
    if (index >= entries_.size()) { return {}; }
    const auto &entry(entries_[index]);
    if (entry.image.data) {
        // opencv image
        return math::Size2(entry.image.cols, entry.image.rows);
    }

    // raw data
    return imgproc::jpegSize(entry.raw.data(), entry.raw.size());
}

} } } // namespace vadstena::vts::opencv
