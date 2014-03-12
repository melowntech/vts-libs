#include <cstdint>

#include <boost/crc.hpp>
#include <boost/format.hpp>

#include "../error.hpp"
#include "./hash-crc.hpp"

namespace vadstena { namespace tilestorage {

namespace {

std::uint32_t calculateHash(const std::string &data)
{
    boost::crc_32_type crc;
    crc.process_bytes(data.data(), data.size());
    return crc.checksum();
}

}

boost::filesystem::path
HashCrcDriver::fileDir_impl(const TileId &tileId, TileFile type
                            , const fs::path &name) const
{
    (void) tileId; (void) type;

    auto hash(calculateHash(name.string()));
    return str(boost::format("%02x/%02x")
               % ((hash >> 24) & 0xff)
               % ((hash >> 16) & 0xff)
               );
}

} } // namespace vadstena::tilestorage
