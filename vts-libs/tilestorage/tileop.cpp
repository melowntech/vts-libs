#include <cstdlib>
#include <stdexcept>

#include <boost/format.hpp>

#include "dbglog/dbglog.hpp"

#include "./tileop.hpp"
#include "./io.hpp"

namespace vadstena { namespace tilestorage {

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
               % tileId.lod % tileId.easting % tileId.northing
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

    if (!(p = parsePart<7>(p, tileId.easting))) { return false; }
    if (*p++ != '-') { return false; }

    if (!(p = parsePart<7>(p, tileId.northing))) { return false; }
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

} } // namespace vadstena::tilestorage
