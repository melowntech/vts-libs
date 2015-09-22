#include <cstring>

#include "dbglog/dbglog.hpp"
#include "utility/binaryio.hpp"

#include "../storage/error.hpp"

#include "./multifile.hpp"

namespace vadstena { namespace vts { namespace multifile {

namespace bin = utility::binaryio;

const Table& Table::versionAtMost(std::uint16_t version
                                  , const boost::filesystem::path &path)
    const
{
    if (this->version > version) {
        LOGTHROW(err1, storage::VersionError)
            << "File " << path
            << " has unsupported version (" << version << ").";
    }

    return *this;
}

const Table& Table::checkEntryCount(Entries::size_type count
                                    , const boost::filesystem::path &path)
    const
{
    if (entries.size() != count) {
        LOGTHROW(err1, storage::FormatError)
            << "File " << path << " must have " << count << " entries.";
    }

    return *this;
}

void writeTable(const Table &table, std::ostream &os)
{
    // writa table
    for (const auto &entry : table) {
        bin::write(os, std::uint32_t(entry.start));
        bin::write(os, std::uint32_t(entry.size));
    }

    // write tail
    bin::write(os, table.magic.data(), table.magic.size());
    bin::write(os, std::uint16_t(table.version));
    bin::write(os, std::uint16_t(table.size()));
}

Table readTable(std::istream &is, const std::string &expectMagic
                , const boost::filesystem::path &path)
{
    // read tail
    std::unique_ptr<char[]> magic(new char[expectMagic.size()]);
    std::uint16_t version;
    std::uint16_t size;

    const auto tailSize(expectMagic.size() + sizeof(version) + sizeof(size));

    is.seekg(-tailSize, std::ios_base::end);

    // read magic
    bin::read(is, magic.get(), expectMagic.size());
    if (std::memcmp(magic.get(), expectMagic.data(), expectMagic.size())) {
        LOGTHROW(err1, storage::BadFileFormat)
            << "File " << path << " is not a VTS atlas file (invalid magic).";
    }

    // read version
    bin::read(is, version);

    // read count first
    bin::read(is, size);

    // seek to table start
    std::uint32_t u32;
    is.seekg(-(tailSize + size * 2 * sizeof(u32)), std::ios_base::end);

    // read table
    Table table(version, expectMagic);
    table.entries.resize(size);

    for (auto &entry : table) {
        bin::read(is, u32); entry.start = u32;
        bin::read(is, u32); entry.size = u32;
    }

    return table;
}

} } } // namespace vadstena::vts::multifile
