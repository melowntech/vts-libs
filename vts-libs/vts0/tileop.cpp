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
#include "math/math.hpp"

#include "tileop.hpp"
#include "io.hpp"

namespace vtslibs { namespace vts0 {

namespace {
    const std::string MetaExt("meta");
    const std::string MeshExt("bin");
    const std::string AtlasExt("jpg");

    const std::string& extension(TileFile type) {
        switch (type) {
        case TileFile::meta: return MetaExt;
        case TileFile::mesh: return MeshExt;
        case TileFile::atlas: return AtlasExt;
        default: throw "Unexpected TileFile value. Go fix your program.";
        }
        throw;
    }

    const char* tileFile(const char *p, TileFile &type) {
        if (!MetaExt.compare(p)) {
            type = TileFile::meta;
            return p + MetaExt.size();
        } else if (!MeshExt.compare(p)) {
            type = TileFile::mesh;
            return p + MeshExt.size();
        } else if (!AtlasExt.compare(p)) {
            type = TileFile::atlas;
            return p + AtlasExt.size();
        }
        return nullptr;
    }
}

std::string asFilename(const TileId &tileId, TileFile type)
{
    return str(boost::format("%s-%07d-%07d.%s")
               % tileId.lod % tileId.x % tileId.y
               % extension(type));
}

namespace {

inline bool isDigit(char c) { return (c >= '0') && (c <= '9'); }

inline char positive(char c) { return c - '0'; }
inline char negative(char c) { return '0' - c; }

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
    if (*p == '-') {
        p = parsePartImpl<(minWidth > 1) ? (minWidth - 1) : 1, negative>
            (p + 1, value);
        if (!value) { return nullptr; }
        return p;
    }
    return parsePartImpl<minWidth, positive>(p, value);
}

} // namespace

bool fromFilename(TileId &tileId, TileFile &type
                  , const std::string &str
                  , std::string::size_type offset)
{
    if (str.size() <= offset) { return false; }

    const char *p(str.c_str() + offset);

    if (!(p = parsePart<1>(p, tileId.lod))) { return false; }
    if (*p++ != '-') { return false; }

    if (!(p = parsePart<1>(p, tileId.x))) { return false; }
    if (*p++ != '-') { return false; }

    if (!(p = parsePart<1>(p, tileId.y))) { return false; }
    if (*p++ != '.') { return false; }

    if (!*p) { return false; }
    auto pp(tileFile(p, type));
    if (!pp) { return false; }
    return !*pp;
}

math::Size2f tileSize(const math::Extents2 &rootExtents, Lod lod)
{
    auto ts(size(rootExtents));
    ts.width /= (1 << lod);
    ts.height /= (1 << lod);
    return ts;
}

TileId fromLl(const Properties &prop, Lod lod, const math::Point2 &ll)
{
    auto ts(tileSize(prop, lod));
    math::Point2 diff(ll - ul(prop.extents));
    return { lod, long(std::round(diff(0) / ts.width))
            , long(-1.0 - std::round(diff(1) / ts.height)) };
}

math::Extents2 aligned(const Properties &prop, Lod lod
                       , math::Extents2 in)
{
    if (in.ll(0) < prop.extents.ll(0)) { in.ll(0) = prop.extents.ll(0); }
    if (in.ll(1) < prop.extents.ll(1)) { in.ll(1) = prop.extents.ll(1); }

    if (in.ur(0) > prop.extents.ur(0)) { in.ur(0) = prop.extents.ur(0); }
    if (in.ur(1) > prop.extents.ur(1)) { in.ur(1) = prop.extents.ur(1); }

    auto ts(tileSize(prop, lod));
    auto orig(ul(prop.extents));
    math::Point2 llDiff(in.ll - orig);
    math::Point2 urDiff(in.ur - orig);

    // LOG(info4) << llDiff << ", " << urDiff;

    math::Point2 llId(llDiff(0) / ts.width, -1.0 - llDiff(1) / ts.height);
    math::Point2 urId(urDiff(0) / ts.width, -1.0 - urDiff(1) / ts.height);

    auto fix([](double &x, bool up) -> void {
            if (math::isInteger(x, 1e-15)) {
                // close enough to be an integer
                x = std::round(x);
            }
            // too far away, floor/ceil
            if (up) {
                if (x < 0) {
                    x = std::floor(x);
                } else {
                    x = std::ceil(x);
                }
            } else {
                if (x < 0) {
                    x = std::ceil(x);
                } else {
                    x = std::floor(x);
                }
            }
        });

    // LOG(info4) << llId << ", " << urId;

    // fix ids
    fix(llId(0), false); fix(llId(1), false);
    fix(urId(0), true); fix(urId(1), true);

    // LOG(info4) << llId << ", " << urId;

    math::Extents2 out
        (math::Point2(orig(0) + llId(0) * ts.width
                      , orig(1) - (llId(1) + 1.0) * ts.height)
         , math::Point2(orig(0) + urId(0) * ts.width
                        , orig(1) - (urId(1) + 1.0) * ts.height));

    // LOG(info4) << std::fixed << "out: " << out;
    return out;
}

math::Extents2 extents(const Properties &prop, const TileId &tileId)
{
    auto ts(tileSize(prop, tileId.lod));

    auto origin(ul(prop.extents));
    return { math::Point2(origin(0) + tileId.x * ts.width
                          , origin(1) - (1 + tileId.y) * ts.height)
            , math::Point2(origin(0) + (tileId.x + 1) * ts.width
                           , origin(1) - tileId.y * ts.height)
            };
}

} } // namespace vtslibs::vts0
