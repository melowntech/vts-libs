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
#include <cstring>

#include "dbglog/dbglog.hpp"

#include "utility/binaryio.hpp"
#include "utility/streams.hpp"
#include "imgproc/imagesize.hpp"

#include "../storage/error.hpp"

#include "atlas.hpp"
#include "multifile.hpp"

namespace vtslibs { namespace vts {

namespace {

const std::string MAGIC("AT");
const std::uint16_t VERSION = 1;

} // namespace

multifile::Table Atlas::readTable(std::istream &is
                                  , const boost::filesystem::path &path)
{
    return multifile::readTable(is, MAGIC, path).versionAtMost(VERSION, path);
}

void Atlas::serialize(std::ostream &os) const
{
    auto table(serialize_impl(os).set(VERSION, MAGIC));
    multifile::writeTable(table, os);
}

void Atlas::deserialize(std::istream &is
                        , const boost::filesystem::path &path)
{
    auto table(readTable(is, path));
    deserialize_impl(is, path, table);
}

math::Size2 Atlas::imageSize(std::size_t index) const
{
    return imageSize_impl(index);
}

void Atlas::write(const boost::filesystem::path &file, std::size_t index) const
{
    utility::ofstreambuf os(file.string());
    write(os, index);
    os.flush();
}

// raw atlas implementation

multifile::Table RawAtlas::serialize_impl(std::ostream &os) const
{
    multifile::Table table;
    auto pos(os.tellp());

    for (const auto &image : images_) {
        using utility::binaryio::write;
        write(os, image.data(), image.size());
        pos = table.add(pos, image.size());
    }

    return table;
}

void RawAtlas::deserialize_impl(std::istream &is
                             , const boost::filesystem::path&
                             , const multifile::Table &table)
{
    Images images;
    for (const auto &entry : table) {
        using utility::binaryio::read;

        is.seekg(entry.start);
        images.emplace_back(entry.size);
        read(is, images.back().data(), entry.size);
    }
    images_.swap(images);
}

math::Size2 RawAtlas::imageSize_impl(std::size_t index) const
{
    if (index >= images_.size()) { return {}; }
    const auto &image(images_[index]);
    return imgproc::imageSize(image.data(), image.size());
}

void RawAtlas::add(const Image &image) {
    images_.push_back(image);
}

void RawAtlas::add(const RawAtlas &other)
{
    images_.insert(images_.end(), other.images_.begin()
                   , other.images_.end());
}

void RawAtlas::write_impl(std::ostream &os, std::size_t index) const
{
    using utility::binaryio::write;

    if (index >= images_.size()) { return; }
    const auto &image(get(index));
    write(os, image.data(), image.size());
}

} } // namespace vtslibs::vts
