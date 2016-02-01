#include <cstring>

#include "dbglog/dbglog.hpp"

#include "utility/binaryio.hpp"
#include "imgproc/jpeg.hpp"

#include "../storage/error.hpp"

#include "./atlas.hpp"
#include "./multifile.hpp"

namespace vadstena { namespace vts {

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
    return imgproc::jpegSize(image.data(), image.size());
}

void RawAtlas::add(const Image &image) {
    images_.push_back(image);
}

void RawAtlas::add(const RawAtlas &other)
{
    images_.insert(images_.end(), other.images_.begin()
                   , other.images_.end());
}

} } // namespace vadstena::vts
