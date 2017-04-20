/**
 * Copyright (c) 2017 Melown Technologies SE
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * *  Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
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
