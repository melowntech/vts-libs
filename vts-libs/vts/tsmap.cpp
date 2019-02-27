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
#include <istream>
#include <sstream>

#include "dbglog/dbglog.hpp"
#include "utility/binaryio.hpp"

#include "tsmap.hpp"

#include "../storage/error.hpp"

namespace vtslibs { namespace vts {

namespace {

const char TM_MAGIC[2] = { 'T', 'M' };

} // namespace

std::string serializeTsMap(const TilesetReferencesList &tsMap)
{
    namespace bin = utility::binaryio;

    std::ostringstream os;
    os.exceptions(std::ostream::failbit | std::ostream::badbit);

    bin::write(os, TM_MAGIC, sizeof(TM_MAGIC));

    // write number of datasets
    bin::write(os, std::uint16_t(tsMap.size()));

    // write all references
    for (const auto &references : tsMap) {
        bin::write(os, std::uint8_t(references.size()));
        for (auto reference : references) {
            bin::write(os, reference);
        }
    }

    return os.str();
}

TilesetReferencesList deserializeTsMap(std::istream &is)
{
    namespace bin = utility::binaryio;

    char magic[sizeof(TM_MAGIC)];
    bin::read(is, magic);
    if (std::memcmp(magic, TM_MAGIC, sizeof(TM_MAGIC))) {
        LOGTHROW(err1, storage::BadFileFormat)
            << "Invalid tile mapping magic.";
    }

    std::uint16_t mapCount;
    bin::read(is, mapCount);

    TilesetReferencesList tsMap;
    tsMap.resize(mapCount);

    for (auto &references : tsMap) {
        std::uint8_t rCount;
        bin::read(is, rCount);
        references.resize(rCount);

        for (auto &reference : references) {
            bin::read(is, reference);
        }
    }

    return tsMap;
}

TilesetReferencesList deserializeTsMap(const std::string &raw)
{
    std::istringstream is(raw);
    is.exceptions(std::istream::failbit | std::istream::badbit);

    return deserializeTsMap(is);
}

} } // namespace vtslibs::vts
