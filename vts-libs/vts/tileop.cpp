#include <cstdlib>
#include <stdexcept>

#include <boost/format.hpp>

#include "dbglog/dbglog.hpp"

#include "./tileop.hpp"
#include "./io.hpp"

namespace vadstena { namespace vts {

namespace {
    const std::string MetaExt("meta");
    const std::string MeshExt("bin");
    const std::string AtlasExt("jpg");

    const std::string& extension(TileFile type) {
        switch (type) {
        case TileFile::meta: return MetaExt;
        case TileFile::mesh: return MeshExt;
        case TileFile::atlas: return AtlasExt;
        }
        throw "unknown tile file type";
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

void misaligned(const Alignment &alignment, long baseTileSize
                , const TileId &tileId)
{
    LOGTHROW(err1, std::domain_error)
        << "Encountered misaligned tile " << tileId
        << " to the grid (" << alignment(0) << ", " << alignment(1)
        << ")/" << baseTileSize << ".";
}

math::Size2f tileSize(const Properties &prop, Lod lod)
{
    auto ts(size(prop.extents));
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
                       , const math::Extents2 &in)
{
    auto ts(tileSize(prop, lod));

    auto llId(fromLl(prop, lod, in.ll));
    auto urId(fromLl(prop, lod, in.ur));

    auto origin(ul(prop.extents));
    ++urId.x;
    --urId.y;
    return { math::Point2(origin(0) + ts.width * llId.x
                          , origin(1) - ts.height * (llId.y - 1))
            , math::Point2(origin(0) + ts.width * urId.x
                           , origin(1) - ts.height * (urId.y - 1)) };
}

} } // namespace vadstena::vts
