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
    const std::string RawMeshExt("rmesh");
    const std::string RawAtlasExt("ratlas");
    const std::string RawNavTileExt("rnavtile");

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

    inline const char* tileFile(const char *p, TileFile &type, bool *raw) {
        if (raw) {*raw = false; }

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
        } else if (!raw) {
            return nullptr;
        }

        // raw part follows
        if (!RawMeshExt.compare(p)) {
            type = TileFile::mesh;
            *raw = true;
            return p + RawMeshExt.size();
        } else  if (!RawAtlasExt.compare(p)) {
            type = TileFile::atlas;
            *raw = true;
            return p + RawAtlasExt.size();
        } else  if (!RawNavTileExt.compare(p)) {
            type = TileFile::navtile;
            *raw = true;
            return p + RawNavTileExt.size();
        }

        return nullptr;
    }

    inline const char* tileFile(const char *p, TileFile &type, bool hasSubFile
                                , bool *raw)
    {
        auto pp(tileFile(p, type, raw));
        if (!pp) { return pp; }
        bool mustHaveSubFile((type == TileFile::atlas) && !(raw && *raw));
        if (hasSubFile != mustHaveSubFile) { return nullptr; }
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
                  , std::string::size_type offset, bool *raw)
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
    auto pp(tileFile(p, type, hasSubFile, raw));

    if (!pp) { return false; }
    return !*pp;
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

} } // namespace vadstena::vts
