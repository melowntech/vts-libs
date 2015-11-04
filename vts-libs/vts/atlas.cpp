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
    multifile::writeTable(serialize_impl(os).set(VERSION, MAGIC), os);
}

void Atlas::deserialize(std::istream &is
                        , const boost::filesystem::path &path)
{
    deserialize_impl(is, path, readTable(is, path));
}

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

std::size_t RawAtlas::area(std::size_t index) const
{
    const auto &image(images_[index]);
    return math::area(imgproc::jpegSize(image.data(), image.size()));
}

} } // namespace vadstena::vts
