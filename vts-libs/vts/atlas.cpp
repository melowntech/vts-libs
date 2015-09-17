#include <cstring>

#include "dbglog/dbglog.hpp"
#include "utility/binaryio.hpp"

#include "../storage/error.hpp"

#include "./atlas.hpp"

namespace vadstena { namespace vts {

namespace bin = utility::binaryio;

namespace {
    const char MAGIC[2] = { 'A', 'T' };
    const std::uint16_t VERSION = 1;
} // namespace

void Atlas::serialize(std::ostream &os) const
{
    auto table(serialize_impl(os));

    // writa table
    for (const auto &entry : table) {
        bin::write(os, std::uint32_t(entry.start));
        bin::write(os, std::uint32_t(entry.size));
    }

    // write tail
    bin::write(os, MAGIC);
    bin::write(os, VERSION);
    bin::write(os, std::uint16_t(table.size()));
}

Atlas::Table Atlas::readTable(std::istream &is
                              , const boost::filesystem::path &path)
{
    // read tail
    char magic[sizeof(MAGIC)];
    std::uint16_t version;
    std::uint16_t size;

    const auto tailSize(sizeof(magic) + sizeof(version) + sizeof(size));

    is.seekg(-tailSize, std::ios_base::end);
    bin::read(is, magic);
    bin::read(is, version);

    if (std::memcmp(magic, MAGIC, sizeof(MAGIC))) {
        LOGTHROW(err1, storage::BadFileFormat)
            << "File " << path << " is not a VTS atlas file.";
    }
    if (version > VERSION) {
        LOGTHROW(err1, storage::VersionError)
            << "File " << path
            << " has unsupported version (" << version << ").";
    }

    // read count first
    bin::read(is, size);

    LOG(info4) << "at: " << is.tellg();
    // seek to table start
    std::uint32_t u32;
    is.seekg(-(tailSize + size * 2 * sizeof(u32)), std::ios_base::end);

    // read table
    Table table;
    table.resize(size);

    for (auto &entry : table) {
        bin::read(is, u32); entry.start = u32;
        bin::read(is, u32); entry.size = u32;
    }

    return table;
}

void Atlas::deserialize(std::istream &is
                        , const boost::filesystem::path &path)
{
    deserialize_impl(is, readTable(is, path));
}

} } // namespace vadstena::vts
