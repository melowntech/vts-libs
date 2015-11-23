#include <cstring>

#include "dbglog/dbglog.hpp"

#include "utility/binaryio.hpp"

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

double Atlas::area(std::size_t index) const
{
    return area_impl(index);
}

} } // namespace vadstena::vts
