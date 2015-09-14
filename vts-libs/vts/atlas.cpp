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
    bin::write(os, MAGIC);
    bin::write(os, VERSION);

    auto table(serialize_impl(os));

    // writa table
    for (const auto &entry : table) {
        bin::write(os, std::uint32_t(entry.start));
        bin::write(os, std::uint32_t(entry.size));
    }
    bin::write(os, std::uint16_t(table.size()));
}

void Atlas::deserialize(std::istream &is
                        , const boost::filesystem::path &path)
{
    char magic[sizeof(MAGIC)];
    std::uint16_t version;

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
    std::uint16_t size;
    std::uint32_t u32;

    is.seekg(sizeof(size), std::ios_base::end);
    bin::read(is, size);

    // seek to table start
    is.seekg(sizeof(size) - size * 2 * sizeof(u32), std::ios_base::end);

    // read table
    Table table;
    table.resize(size);

    for (auto &entry : table) {
        bin::read(is, u32); entry.start = u32;
        bin::read(is, u32); entry.size = u32;
    }

    deserialize_impl(is, table);
}

} } // namespace vadstena::vts
