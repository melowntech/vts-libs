#include <cstring>

#include "dbglog/dbglog.hpp"

#include "utility/binaryio.hpp"

#include "../storage/error.hpp"

#include "./atlas.hpp"
#include "./multifile.hpp"

namespace vadstena { namespace vts {

namespace {

const std::string MAGIC("AT");
const std::uint16_t VERSION = 2;

const std::string PROPERTIES_MAGIC("PR");
const std::uint16_t PROPERTIES_VERSION = 1;

} // namespace

Atlas::Table::Table(const multifile::Table &src)
    : multifile::Table(src)
{
    if ((version > 1) && !empty()) {
        // get entry for properties (last entry) and erase from table
        properties = entries.back();
        entries.resize(entries.size() - 1);
    }
}

Atlas::Table Atlas::readTable(std::istream &is
                              , const boost::filesystem::path &path)
{
    return multifile::readTable(is, MAGIC, path).versionAtMost(VERSION, path);
}

namespace {

Atlas::Properties::list deserializeProperties
(std::istream &is, const multifile::Table::Entry &entry
 , const boost::filesystem::path &path)
{
    namespace bin = utility::binaryio;

    if (!entry) { return {}; }

    // seek to start of properties
    is.seekg(entry.start, std::ios_base::beg);

    std::unique_ptr<char[]> magic(new char[PROPERTIES_MAGIC.size()]);
    std::uint16_t version;
    std::uint16_t size;

    bin::read(is, magic.get(), PROPERTIES_MAGIC.size());
    if (std::memcmp(magic.get(), PROPERTIES_MAGIC.data()
                    , PROPERTIES_MAGIC.size()))
    {
        LOGTHROW(err1, storage::BadFileFormat)
            << "File " << path << " contains wrong properties magic.";
    }

    // read version
    bin::read(is, version);

    if (version > 1) {
        LOGTHROW(err1, storage::VersionError)
            << "File " << path
            << " has unsupported properties version (" << version << ").";
    }

    // read count
    bin::read(is, size);

    Atlas::Properties::list properties(size);
    for (auto &p : properties) {
        double apa;
        bin::read(is, apa);
        p.apparentPixelArea = apa;
    }

    return properties;
}

multifile::Table::Entry
serializeProperties(std::ostream &os
                    , const Atlas::Properties::list &properties)
{
    namespace bin = utility::binaryio;

    auto pos(os.tellp());

    // write header
    bin::write(os, PROPERTIES_MAGIC);
    bin::write(os, PROPERTIES_VERSION);

    bin::write(os, std::uint16_t(properties.size()));
    for (const auto p : properties) {
        bin::write(os, double(p.apparentPixelArea));
    }

    auto npos(os.tellp());
    return multifile::Table::Entry(pos, npos - pos);
}

}

void Atlas::serialize(std::ostream &os) const
{
    auto table(serialize_impl(os).set(VERSION, MAGIC));
    table.add(serializeProperties(os, properties_));
    multifile::writeTable(table, os);
}

void Atlas::deserialize(std::istream &is
                        , const boost::filesystem::path &path)
{
    auto table(readTable(is, path));
    properties_ = deserializeProperties(is, table.properties, path);
    deserialize_impl(is, path, table);
}

void Atlas::properties(std::size_t index, const Properties &properties)
{
    if (index >= properties_.size()) {
        properties_.resize(index + 1);
    }
    properties_[index] = properties;
}

Atlas::Properties Atlas::properties(std::size_t index) const
{
    if (index >= properties_.size()) { return {}; }
    return properties_[index];
}

double Atlas::area(std::size_t index) const
{
    auto area(area_impl(index));
    return area * properties(index).apparentPixelArea;
}

} } // namespace vadstena::vts
