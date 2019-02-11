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
#include <opencv2/highgui/highgui.hpp>

#include "dbglog/dbglog.hpp"

#include "utility/binaryio.hpp"

#include "imgproc/readimage.hpp"

#include "../../storage/error.hpp"

#include "atlas.hpp"

namespace fs = boost::filesystem;

namespace vtslibs { namespace vts { namespace opencv {

namespace {

std::vector<unsigned char> mat2jpeg(const cv::Mat &mat, int quality)
{
    std::vector<unsigned char> buf;
    if (quality > 0) {
        // store as jpeg
        cv::imencode(".jpg", mat, buf
                     , { cv::IMWRITE_JPEG_QUALITY, quality });
    } else {
        // store as png
        cv::imencode(".png", mat, buf
                     , { cv::IMWRITE_PNG_COMPRESSION, 9 });
    }
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

Atlas::Atlas(const vts::Atlas &atlas, int textureQuality)
    : quality_(textureQuality)
{
    if (const auto *in = dynamic_cast<const RawAtlas*>(&atlas)) {
        for (const auto &image : in->get()) {
            images_.push_back(jpeg2mat(image));
        }
        return;
    }

    if (const auto *in = dynamic_cast<const opencv::Atlas*>(&atlas)) {
        images_ = in->get();
        return;
    }

    if (const auto *in = dynamic_cast<const HybridAtlas*>(&atlas)) {
        for (std::size_t i(0), e(in->size()); i != e; ++i) {
            images_.push_back(in->get(i));
        }
        return;
    }

    LOGTHROW(err1, storage::Unimplemented)
        << "Cannot extract images from provided atlas.";
}

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

void Atlas::write_impl(std::ostream &os, std::size_t index) const
{
    if (index >= images_.size()) { return; }
    const auto &image(get(index));

    using utility::binaryio::write;
    auto buf(mat2jpeg(image, quality_));
    write(os, buf.data(), buf.size());
}

void Atlas::append(const Atlas &atlas)
{
    images_.insert(images_.end(), atlas.images_.begin(), atlas.images_.end());
}

HybridAtlas::HybridAtlas(std::size_t count, const RawAtlas &rawAtlas
                         , int quality)
    : quality_(quality)
{
    for (std::size_t i(0), e(std::min(rawAtlas.size(), count)); i != e; ++i) {
        entries_.emplace_back(rawAtlas.get(i));
    }
}

HybridAtlas::HybridAtlas(std::size_t count, const HybridAtlas &hybridAtlas
                         , int quality)
    : quality_(quality)
{
    for (std::size_t i(0), e(std::min(hybridAtlas.size(), count)); i != e; ++i)
    {
        entries_.push_back(hybridAtlas.entry(i));
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

void HybridAtlas::add(const fs::path &file)
{
    entries_.emplace_back(file);
}

HybridAtlas::Image HybridAtlas::imageFromRaw(const Raw &raw)
{
    return jpeg2mat(raw);
}

HybridAtlas::Raw HybridAtlas::rawFromImage(const Image &image, int quality)
{
    return mat2jpeg(image, quality);
}

HybridAtlas::Image HybridAtlas::imageFromFile(const fs::path &file)
{
    return imgproc::readImage(file);
}

void HybridAtlas::set(std::size_t index, const Image &image)
{
    if (entries_.size() <= index) {
        entries_.resize(index + 1);
    }
    auto &entry(entries_[index]);
    entry.raw.clear();
    entry.image = image;
    entry.file.clear();
}

void HybridAtlas::set(std::size_t index, const Raw &raw)
{
    if (entries_.size() <= index) {
        entries_.resize(index + 1);
    }
    auto &entry(entries_[index]);
    entry.raw = raw;
    entry.image = cv::Mat();
    entry.file.clear();
}

void HybridAtlas::set(std::size_t index, const fs::path &file)
{
    if (entries_.size() <= index) {
        entries_.resize(index + 1);
    }
    auto &entry(entries_[index]);
    entry.raw.clear();
    entry.image = cv::Mat();
    entry.file = file;
}

HybridAtlas::Image HybridAtlas::get(std::size_t index) const
{
    const auto &entry(entries_[index]);
    // redirect
    if (entry.source) { return get(*entry.source); }

    if (entry.image.data) { return entry.image; }
    if (!entry.file.empty()) { return imgproc::readImage(entry.file); }
    return imageFromRaw(entry.raw);
}

void HybridAtlas::append(const HybridAtlas &atlas)
{
    entries_.insert(entries_.end(), atlas.entries_.begin()
                    , atlas.entries_.end());
}

void HybridAtlas::append(const opencv::Atlas &atlas)
{
    for (const auto &image : atlas.get()) {
        entries_.emplace_back(image);
    }
}

HybridAtlas::Entry HybridAtlas::entry(std::size_t index) const
{
    return entries_[index];
}

void HybridAtlas::add(const Entry &entry)
{
    entries_.push_back(entry);
}

multifile::Table HybridAtlas::serialize_impl(std::ostream &os) const
{
    multifile::Table table;
    auto pos(os.tellp());

    std::size_t index(0);
    for (const auto &entry : entries_) {
        using utility::binaryio::write;
        if (entry.image.data) {
            auto buf(mat2jpeg(entry.image, quality_));
            write(os, buf.data(), buf.size());
            pos = table.add(pos, buf.size());
        } else if (!entry.file.empty()) {
            auto buf(mat2jpeg(imageFromFile(entry.file), quality_));
            write(os, buf.data(), buf.size());
            pos = table.add(pos, buf.size());
        } else if (entry.source) {
            if (*entry.source >= index) {
                LOGTHROW(err1, storage::BadFileFormat)
                    << "Invalid source index " << *entry.source << ".";
            }
            // clone table entry
            table.add(table.entries[*entry.source]);
        } else {
            write(os, entry.raw.data(), entry.raw.size());
            pos = table.add(pos, entry.raw.size());
        }
        ++index;
    }

    return table;
}

void HybridAtlas::deserialize_impl(std::istream &is
                                   , const boost::filesystem::path &path
                                   , const multifile::Table &table)
{
    Entries entries;
    for (const auto &entry : table) {
        using utility::binaryio::read;

        is.seekg(entry.start);
        std::vector<unsigned char> buf;
        buf.resize(entry.size);
        read(is, buf.data(), buf.size());

        entries.emplace_back(jpeg2mat(buf, &entry, &path));
    }
    entries_.swap(entries);
}

math::Size2 HybridAtlas::imageSize_impl(std::size_t index) const
{
    if (index >= entries_.size()) { return {}; }
    const auto &entry(entries_[index]);

    // redirect
    if (entry.source) { return imageSize_impl(*entry.source); }

    if (entry.image.data) {
        // opencv image
        return math::Size2(entry.image.cols, entry.image.rows);
    }

    if (!entry.file.empty()) {
        return imgproc::imageSize(entry.file);
    }

    // raw data
    return imgproc::imageSize(entry.raw.data(), entry.raw.size());
}

void HybridAtlas::write_impl(std::ostream &os, std::size_t index) const
{
    if (index >= entries_.size()) { return; }
    const auto &e(entry(index));

    using utility::binaryio::write;
    if (e.image.data) {
        auto buf(mat2jpeg(e.image, quality_));
        write(os, buf.data(), buf.size());
    } else if (!e.file.empty()) {
        const auto mat(imageFromFile(e.file));
        if (!mat.data) {
            LOGTHROW(err1, storage::BadFileFormat)
                << "Cannot read image file from " << e.file << ".";
        }
        auto buf(mat2jpeg(mat, quality_));
        write(os, buf.data(), buf.size());
    } else if (e.source) {
        write_impl(os, *e.source);
    } else {
        write(os, e.raw.data(), e.raw.size());
    }
}

HybridAtlas::HybridAtlas(const Atlas &atlas, int textureQuality)
    : quality_(textureQuality)
{
    if (const auto *in = dynamic_cast<const RawAtlas*>(&atlas)) {
        for (const auto &image : in->get()) {
            entries_.emplace_back(image);
        }
        return;
    }

    if (const auto *in = dynamic_cast<const opencv::Atlas*>(&atlas)) {
        for (const auto &image : in->get()) {
            entries_.emplace_back(image);
        }
        return;
    }

    if (const auto *in = dynamic_cast<const HybridAtlas*>(&atlas)) {
        entries_ = in->entries_;
        return;
    }

    LOGTHROW(err1, storage::Unimplemented)
        << "Cannot extract images from provided atlas.";
}

void HybridAtlas::duplicate()
{
    if (empty()) { return; }

    std::size_t index(size() - 1);
    while (entries_[index].source) {
        index = *entries_[index].source;
    }

    add(Entry(index));
}

} } } // namespace vtslibs::vts::opencv
