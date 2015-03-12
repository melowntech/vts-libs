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

std::string filePath(const TileId &tileId, TileFile type)
{
    return str(boost::format("%s-%07d-%07d.%s")
               % tileId.lod % tileId.easting % tileId.northing
               % extension(type));
}

bool fromFilename(TileId &tileId, TileFile &type
                  , const std::string &str
                  , std::string::size_type offset)
{
    if (str.size() <= offset) { return false; }

    const char *p(str.c_str() + offset);
    char *e(nullptr);

    tileId.lod = std::strtol(p, &e, 10);
    if (*e != '-') { return false; }
    p = e + 1;

    tileId.easting = std::strtol(p, &e, 10);
    if (*e != '-') { return false; }
    p = e + 1;

    tileId.northing = std::strtol(p, &e, 10);
    if (*e != '.') { return false; }
    p = e + 1;

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
