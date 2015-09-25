#include "./navtile.hpp"

namespace vadstena { namespace vts {

namespace {
    const std::string MAGIC("NT");
    const std::uint16_t VERSION = 1;
} // namespace

multifile::Table NavTile::readTable(std::istream &is
                                  , const boost::filesystem::path &path)
{
    return multifile::readTable(is, MAGIC, path)
        .versionAtMost(VERSION, path)
        .checkEntryCount(2, path);
}

void NavTile::serialize(std::ostream &os) const
{
    auto table(serialize_impl(os).set(VERSION, MAGIC));

    // write mask
    auto pos(os.tellp());
    coverageMask_.dump(os);
    table.add(pos, os.tellp() - pos);

    multifile::writeTable(table, os);
}

void NavTile::deserialize(const HeightRange &heightRange, std::istream &is
                        , const boost::filesystem::path &path)
{
    auto table(readTable(is, path));
    deserialize_impl(heightRange, is, path, table);

    // read mask
    is.seekg(table.entries[1].start);
    coverageMask_.load(is);
}

} } // namespace vadstena::vts
