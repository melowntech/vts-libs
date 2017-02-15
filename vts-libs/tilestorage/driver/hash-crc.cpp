#include <cstdint>

#include <boost/crc.hpp>
#include <boost/format.hpp>

#include "../../storage/error.hpp"
#include "./hash-crc.hpp"

namespace vtslibs { namespace tilestorage {

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


const std::string HashCrcDriver::help
(R"RAW(Filesystem-based storage driver with two-level directory hierarchy.
Files are in directory name hex(CRC-32[MSB])/hex(CRC-32[MSB-1]) where:
    * CRC-32 is CRC-32 of filename
    * MSB is most significant byte of CRC-32
    * hex is hexadecimal representation of a byte (zero padded)

Example: filename 11-0486112-5613088.jpg
   * CRC-32 is 0xc56b5904
   * MSB is c5, MSB-1 is 6b
   * directory is c5/6b
   * full path is c5/6b/11-0486112-5613088.jpg
)RAW");

} } // namespace vtslibs::tilestorage
