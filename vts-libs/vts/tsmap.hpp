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

#ifndef vtslibs_vts_tsmap_hpp_included_
#define vtslibs_vts_tsmap_hpp_included_

#include <cstdint>
#include <vector>
#include <iosfwd>
#include <string>

namespace vtslibs { namespace vts {

typedef std::uint16_t TilesetReference;
typedef std::vector<TilesetReference> TilesetReferences;
typedef std::vector<TilesetReferences> TilesetReferencesList;

/** Serialize tileset mapping to string.
 */
std::string serializeTsMap(const TilesetReferencesList &tsMap);

/** Deserialize tileset mapping from a string.
 */
TilesetReferencesList deserializeTsMap(const std::string &raw);

/** Deserialize tileset mapping from an input stream.
 */
TilesetReferencesList deserializeTsMap(std::istream &is);

} } // namespace vtslibs::vts

#endif // vtslibs_vts_tsmap_included_
