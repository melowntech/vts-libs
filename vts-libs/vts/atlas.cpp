#include <cstring>

#include "dbglog/dbglog.hpp"

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
    deserialize_impl(is, readTable(is, path));
}

} } // namespace vadstena::vts
