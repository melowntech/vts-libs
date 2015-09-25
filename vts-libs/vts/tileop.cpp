#include <cstdlib>
#include <stdexcept>

#include <boost/format.hpp>

#include "dbglog/dbglog.hpp"
#include "math/math.hpp"

#include "./tileop.hpp"
#include "./io.hpp"

namespace vadstena { namespace vts {

namespace {
    const std::string MetaExt("meta");
    const std::string MeshExt("bin");
    const std::string AtlasExt("jpg");
    const std::string NavTileExt("nav");

    inline const std::string& extension(TileFile type) {
        switch (type) {
        case TileFile::meta: return MetaExt;
        case TileFile::mesh: return MeshExt;
        case TileFile::atlas: return AtlasExt;
        case TileFile::navtile: return NavTileExt;
        default: throw "Unexpected TileFile value. Go fix your program.";
        }
        throw;
    }

    inline const char* tileFile(const char *p, TileFile &type) {
        if (!MetaExt.compare(p)) {
            type = TileFile::meta;
            return p + MetaExt.size();
        } else if (!MeshExt.compare(p)) {
            type = TileFile::mesh;
            return p + MeshExt.size();
        } else if (!AtlasExt.compare(p)) {
            type = TileFile::atlas;
            return p + AtlasExt.size();
        } else if (!NavTileExt.compare(p)) {
            type = TileFile::navtile;
            return p + NavTileExt.size();
        }
        return nullptr;
    }

    inline const char* tileFile(const char *p, TileFile &type, bool hasSubFile)
    {
        auto pp(tileFile(p, type));
        if (!pp) { return pp; }
        if (hasSubFile && (type != TileFile::atlas)) { return nullptr; }
        if (!hasSubFile && (type == TileFile::atlas)) { return nullptr; }
        return pp;
    }
}

std::string asFilename(const TileId &tileId, TileFile type)
{
    return str(boost::format("%s-%07d-%07d.%s")
               % tileId.lod % tileId.x % tileId.y
               % extension(type));
}

std::string fileTemplate(TileFile type, boost::optional<unsigned int> revision)
{

    std::string ext(extension(type));
    if (revision) {
        ext = str(boost::format("%s?%s") % ext % *revision);
    }

    if (type == TileFile::atlas) {
        return str(boost::format("{lod}-{x}-{y}-{sub}.%s") % ext);
    }

    return str(boost::format("{lod}-{x}-{y}.%s") % ext);
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
                  , std::string::size_type offset)
{
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
    auto pp(tileFile(p, type, hasSubFile));

    if (!pp) { return false; }
    return !*pp;
}

} } // namespace vadstena::vts
