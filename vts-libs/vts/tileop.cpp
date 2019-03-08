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
#include <cstdlib>
#include <stdexcept>

#include <boost/format.hpp>

#include "dbglog/dbglog.hpp"
#include "utility/raise.hpp"
#include "math/math.hpp"

#include "tileop.hpp"
#include "io.hpp"

namespace vtslibs { namespace vts {

namespace {
    const std::string MetaExt("meta");
    const std::string MeshExt("bin");
    const std::string AtlasExt("jpg");
    const std::string NavTileExt("nav");
    const std::string Meta2dExt("2dmeta");
    const std::string MaskExt("mask");
    const std::string OrthoExt("ortho");
    const std::string CreditsExt("credits");

    const std::string RawMeshExt("rmesh");
    const std::string RawAtlasExt("ratlas");
    const std::string RawNavTileExt("rnavtile");

    const std::string DebugMaskExt("mask.dbg");
    const std::string DebugMetaExt("meta.dbg");

    inline const std::string& extension(TileFile type, FileFlavor flavor)
    {
        switch (type) {
        case TileFile::meta:
            switch (flavor) {
            case FileFlavor::regular: return MetaExt;
            case FileFlavor::raw: break;
            case FileFlavor::debug: return DebugMetaExt;
            }
            break;
        case TileFile::mesh:
            switch (flavor) {
            case FileFlavor::regular: return MeshExt;
            case FileFlavor::raw: return RawMeshExt;
            case FileFlavor::debug: break;
            }
            break;
        case TileFile::atlas:
            switch (flavor) {
            case FileFlavor::regular: return AtlasExt;
            case FileFlavor::raw: return RawAtlasExt;
            case FileFlavor::debug: break;
            }
            break;
        case TileFile::navtile:
            switch (flavor) {
            case FileFlavor::regular: return NavTileExt;
            case FileFlavor::raw: return RawNavTileExt;
            case FileFlavor::debug: break;
            }
            break;
        case TileFile::meta2d: return Meta2dExt;
        case TileFile::mask:
            switch (flavor) {
            case FileFlavor::regular: return MaskExt;
            case FileFlavor::raw: break;
            case FileFlavor::debug: return DebugMaskExt;
            }
            break;
        case TileFile::ortho: return OrthoExt;
        case TileFile::credits: return CreditsExt;
        default: break;
        }

        utility::raise<std::string>
            ("Unexpected TileFile value <%s>/<%s>. Go fix your program."
             , type, flavor);
        throw;
    }

    inline const char* tileFile(const char *p, TileFile &type
                                , FileFlavor *flavor)
    {
        if (flavor) { *flavor = FileFlavor::regular; }
#define HANDLE_EXT(EXT, TYPE)                   \
        if (!EXT.compare(p)) {                  \
            type = TileFile::TYPE;              \
            return p + EXT.size();              \
        }

        HANDLE_EXT(MetaExt, meta)
        HANDLE_EXT(MeshExt, mesh)
        HANDLE_EXT(AtlasExt, atlas)
        HANDLE_EXT(NavTileExt, navtile)
        HANDLE_EXT(Meta2dExt, meta2d)
        HANDLE_EXT(MaskExt, mask)
        HANDLE_EXT(OrthoExt, ortho)
        HANDLE_EXT(CreditsExt, credits)
        if (!flavor) { return nullptr; }
#undef HANDLE_EXT

#define HANDLE_EXT(EXT, TYPE)                   \
        if (!EXT.compare(p)) {                  \
            type = TileFile::TYPE;              \
            *flavor = FileFlavor::raw;          \
            return p + EXT.size();              \
        }

        HANDLE_EXT(RawMeshExt, mesh)
        HANDLE_EXT(RawAtlasExt, atlas)
        HANDLE_EXT(RawNavTileExt, navtile)
#undef HANDLE_EXT

#define HANDLE_EXT(EXT, TYPE)                   \
        if (!EXT.compare(p)) {                  \
            type = TileFile::TYPE;              \
            *flavor = FileFlavor::debug;        \
            return p + EXT.size();              \
        }

        HANDLE_EXT(DebugMetaExt, meta)
        HANDLE_EXT(DebugMaskExt, mask)
#undef HANDLE_EXT

        return nullptr;
    }

    inline const char* tileFile(const char *p, TileFile &type, bool hasSubFile
                                , FileFlavor *flavor)
    {
        auto pp(tileFile(p, type, flavor));
        if (!pp) { return pp; }

        // subfile is mandatory for given non-raw files
        bool mustHaveSubFile([&]() -> bool {
                switch (type) {
                case TileFile::atlas:
                case TileFile::ortho:
                    return !(flavor && (*flavor != FileFlavor::regular));
                    break;

                default: break;
                }
                return false;
            }());

        if (hasSubFile != mustHaveSubFile) { return nullptr; }
        return pp;
    }
}

std::string asFilename(const TileId &tileId, TileFile type, FileFlavor flavor)
{
    return str(boost::format("%s-%07d-%07d.%s")
               % tileId.lod % tileId.x % tileId.y
               % extension(type, flavor));
}

std::string fileTemplate(TileFile type, FileFlavor flavor
                         , const boost::optional<unsigned int> &revision)
{
    std::string ext(extension(type, flavor));
    if (revision) {
        ext = str(boost::format("%s?%s") % ext % *revision);
    }

    switch (type) {
    case TileFile::atlas:
    case TileFile::ortho:
        return str(boost::format("{lod}-{x}-{y}-{sub}.%s") % ext);

    default: break;
    }

    return str(boost::format("{lod}-{x}-{y}.%s") % ext);
}

std::string filePath(TileFile type, const TileId &tileId
                     , const boost::optional<unsigned int> &subfile
                     , const boost::optional<unsigned int> &revision)
{
    std::string ext(extension(type, FileFlavor::regular));
    if (revision) {
        ext = str(boost::format("%s?%s") % ext % *revision);
    }

    switch (type) {
    case TileFile::atlas:
    case TileFile::ortho:
        return str(boost::format("%d-%d-%d-%d.%s")
                   % tileId.lod % tileId.x % tileId.y
                   % (subfile ? *subfile : 0) % ext);

    default: break;
    }

    return str(boost::format("%d-%d-%d.%s")
               % tileId.lod % tileId.x % tileId.y % ext);
}

namespace {

inline bool isDigit(char c) { return (c >= '0') && (c <= '9'); }

inline char positive(char c) { return c - '0'; }

template <unsigned int minWidth, char(*getter)(char), typename T>
inline const char* parsePartImpl(const char *p, T &value)
{
    bool prefix = false;
    char c(p[0]);
    switch (c) {
    case '-': case '+': return nullptr;
    case '0': prefix = true;
    }

    value = 0;

    const char *e(p);
    while (isDigit(c)) {
        value *= 10;
        value += getter(c);
        c = *++e;
    }

    auto dist(e - p);
    if (dist < minWidth) { return nullptr; }
    if (prefix && (dist > minWidth)) { return nullptr; }
    return e;
}

template <unsigned int minWidth, typename T>
inline const char* parsePart(const char *p, T &value)
{
    // only positive numbers are allowed
    return parsePartImpl<minWidth, positive>(p, value);
}

} // namespace

bool fromFilename(TileId &tileId, TileFile &type, unsigned int &subTileFile
                  , const std::string &str
                  , std::string::size_type offset, FileFlavor *flavor)
{
    // TODO: use parseTileIdPrefix
    if (str.size() <= offset) { return false; }

    const char *p(str.c_str() + offset);

    if (!(p = parsePart<1>(p, tileId.lod))) { return false; }
    if (*p++ != '-') { return false; }

    if (!(p = parsePart<1>(p, tileId.x))) { return false; }
    if (*p++ != '-') { return false; }

    if (!(p = parsePart<1>(p, tileId.y))) { return false; }

    bool hasSubFile(false);
    if (*p == '-') {
        // we have sub-file
        ++p;
        if (!(p = parsePart<1>(p, subTileFile))) { return false; }
        hasSubFile = true;
    }

    if (*p++ != '.') { return false; }

    if (!*p) { return false; }
    auto pp(tileFile(p, type, hasSubFile, flavor));

    if (!pp) { return false; }
    return !*pp;
}

const char* parseTileIdPrefix(TileId &tileId, const std::string &str
                              , boost::optional<unsigned int> *subTileFile
                              , std::string::size_type offset)
{
    if (str.size() <= offset) { return nullptr; }

    const char *p(str.c_str() + offset);

    if (!(p = parsePart<1>(p, tileId.lod))) { return nullptr; }
    if (*p++ != '-') { return nullptr; }

    if (!(p = parsePart<1>(p, tileId.x))) { return nullptr; }
    if (*p++ != '-') { return nullptr; }

    if (!(p = parsePart<1>(p, tileId.y))) { return nullptr; }

    if (*p == '-') {
        if (!subTileFile) { return nullptr; }

        // we have sub-file
        ++p;
        unsigned int stf(0);
        if (!(p = parsePart<1>(p, stf))) { return nullptr; }
        *subTileFile = stf;
    }

    if (*p++ != '.') { return nullptr; }

    if (!*p) { return nullptr; }
    return p;
}

TileId commonAncestor(Lod lod, TileRange range)
{
    // sanity check
    if (!valid(range)) { return {}; }

    // iterative approach: tranverse the tree up until we hit the same parent
    // point tile
    while ((range.ll(0) != range.ur(0)) || (range.ll(1) != range.ur(1))) {
        range.ll(0) >>= 1;
        range.ll(1) >>= 1;
        range.ur(0) >>= 1;
        range.ur(1) >>= 1;
        --lod;
    }
    return { lod, range.ll(0), range.ll(1) };
}

BorderCondition inside(const LodTileRange &range, const TileId &tileId
                       , bool above)
{
    if (tileId.lod < range.lod) {
        // TODO: handle above flag
        (void) above;
        return false;
    }

    auto tr((tileId.lod == range.lod)
            ? range.range
            : childRange(range.range, tileId.lod - range.lod));

    // outside?
    if ((tileId.x < tr.ll(0)) || (tileId.x > tr.ur(0))
        || (tileId.y < tr.ll(1)) || (tileId.y > tr.ur(1)))
    {
        return false;
    }

    int flags(BorderCondition::inside);

    // border?
    if (tileId.x == tr.ll(0)) { flags |= BorderCondition::left; }
    if (tileId.x == tr.ur(0)) { flags |= BorderCondition::right; }
    if (tileId.y == tr.ll(1)) { flags |= BorderCondition::top; }
    if (tileId.y == tr.ur(1)) { flags |= BorderCondition::bottom; }

    return { flags };
}

math::Extents2
inflateTileExtents(const math::Extents2 &extents
                   , double margin
                   , const BorderCondition &borderCondition
                   , double borderMargin)
{
    auto choose([&](int flag)
    {
        return (borderCondition.check(flag) ? borderMargin : margin);
    });

    // tile size
    auto ts(math::size(extents));
    return math::Extents2
        (extents.ll(0) - ts.width * choose(BorderCondition::left)
         , extents.ll(1) - ts.height * choose(BorderCondition::bottom)
         , extents.ur(0) + ts.width * choose(BorderCondition::right)
         ,  extents.ur(1) + ts.height * choose(BorderCondition::top));
}

bool inside(const Ranges &ranges, const TileId &tileId)
{
    if (const auto *tileRange = ranges.tileRange(tileId.lod, std::nothrow)) {
        return math::inside(*tileRange, tileId.x, tileId.y);
    }
    return false;
}

bool under(const Ranges &ranges, const TileId &tileId)
{
    // first, check for tile being in above or at max lod
    auto max(ranges.lodRange().max);
    if (tileId.lod <= max) { return inside(ranges, tileId); }

    // tile is somewhere under the max lod, test its parent at max lod
    return inside(ranges, parent(tileId, tileId.lod - max));
}

bool overlaps(const Ranges &ranges, const LodTileRange &range)
{
    if (const auto *tileRange = ranges.tileRange(range.lod, std::nothrow)) {
        // calculate overlap and return its validity
        return math::valid(vts::tileRangesIntersect
                           (*tileRange, range.range, std::nothrow));
    }
    return false;
}

} } // namespace vtslibs::vts
