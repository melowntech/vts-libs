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

#include <cstring>
#include <sstream>

#include "dbglog/dbglog.hpp"
#include "utility/binaryio.hpp"

#include "./tsmap.hpp"

#include "../storage/error.hpp"

namespace vtslibs { namespace vts {

namespace {

const char TM_MAGIC[2] = { 'T', 'M' };

} // namespace

std::string serializeTsMap(const TilesetReferencesList &tsMap)
{
    using utility::binaryio::write;
    std::ostringstream os;
    os.exceptions(std::ostream::failbit | std::ostream::badbit);

    write(os, TM_MAGIC, sizeof(TM_MAGIC));

    // write number of datasets
    write(os, std::uint16_t(tsMap.size()));

    // write all references
    for (const auto &references : tsMap) {
        write(os, std::uint8_t(references.size()));
        for (auto reference : references) {
            write(os, reference);
        }
    }

    return os.str();
}

TilesetReferencesList deserializeTsMap(const std::string &raw)
{
    using utility::binaryio::read;
    std::istringstream is(raw);
    is.exceptions(std::istream::failbit | std::istream::badbit);

    char magic[sizeof(TM_MAGIC)];
    read(is, magic);
    if (std::memcmp(magic, TM_MAGIC, sizeof(TM_MAGIC))) {
        LOGTHROW(err1, storage::BadFileFormat)
            << "Invalid tile mapping magic.";
    }

    std::uint16_t mapCount;
    read(is, mapCount);

    TilesetReferencesList tsMap;
    tsMap.resize(mapCount);

    for (auto &references : tsMap) {
        std::uint8_t rCount;
        read(is, rCount);
        references.resize(rCount);

        for (auto &reference : references) {
            read(is, reference);
        }
    }

    return tsMap;
}

} } // namespace vtslibs::vts
