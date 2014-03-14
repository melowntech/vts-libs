#include <queue>
#include <bitset>
#include <iostream>

#include "dbglog/dbglog.hpp"
#include "utility/binaryio.hpp"
#include "math/geometry.hpp"

#include "./error.hpp"
#include "./tileop.hpp"
#include "./io.hpp"
#include "./metatile.hpp"

namespace vadstena { namespace tilestorage {

namespace detail {

double triangleArea(const math::Point3 &a, const math::Point3 &b,
                    const math::Point3 &c)
{
    return norm_2(math::crossProduct(b - a, c - a)) * 0.5;
}

double pixelSize(const geometry::Obj &mesh, const math::Size2 &atlasSize)
{
    if (mesh.facets.empty()) return detail::invalidPixelSize;

    // calculate the total area of the faces in both the XYZ and UV spaces
    double xyzArea(0), uvArea(0);
    for (const auto &face : mesh.facets)
    {
        xyzArea += triangleArea(mesh.vertices[face.v[0]],
                                mesh.vertices[face.v[1]],
                                mesh.vertices[face.v[2]]);
        uvArea += triangleArea(mesh.texcoords[face.t[0]],
                               mesh.texcoords[face.t[1]],
                               mesh.texcoords[face.t[2]]);
    }
    uvArea *= atlasSize.width * atlasSize.height;

    return sqrt(xyzArea / uvArea);
}

} // namespace

void MetaNode::calcParams(const geometry::Obj &mesh,
                          const math::Size2 &atlasSize)
{
    if (mesh.facets.empty()) return;

    // calculate Z range
    zmin = INFINITY; zmax = -INFINITY;
    for (const auto &vertex : mesh.vertices)
    {
        if (vertex(2) < zmin) zmin = vertex(2);
        if (vertex(2) > zmax) zmax = vertex(2);
    }

    // calculate texture resolution
    double ps(detail::pixelSize(mesh, atlasSize));
    pixelSize[0][0] = pixelSize[0][1] = ps;
    pixelSize[1][0] = pixelSize[1][1] = ps;
}


void MetaNode::dump(std::ostream &f) const
{
    namespace bin = utility::binaryio;

    bin::write(f, int16_t(std::floor(zmin)));
    bin::write(f, int16_t(std::ceil(zmax)));

    for (int i = 0; i < 2; i++)
    for (int j = 0; j < 2; j++) {
        bin::write(f, pixelSize[i][j]);
    }

    for (int i = 0; i < HMSize; i++)
    for (int j = 0; j < HMSize; j++) {
        bin::write(f, int16_t(round(heightmap[i][j])));
    }
}

namespace {
const char METATILE_IO_MAGIC[8] = {  'M', 'E', 'T', 'A', 'T', 'I', 'L', 'E' };

const unsigned METATILE_IO_VERSION = 1;

void loadMetatileTree(long baseTileSize, const TileId &tileId
                      , const MetaNodeLoader &loader, std::istream &f)
{
    using utility::binaryio::read;

    MetaNode node;

    std::int16_t zmin, zmax;
    read(f, zmin);
    read(f, zmax);
    node.zmin = zmin;
    node.zmax = zmax;

    for (int i = 0; i < 2; ++i) {
        for (int j = 0; j < 2; ++j) {
            read(f, node.pixelSize[i][j]);
        }
    }

    for (int i = 0; i < MetaNode::HMSize; ++i) {
        for (int j = 0; j < MetaNode::HMSize; ++j) {
            std::int16_t value;
            read(f, value);
            node.heightmap[i][j] = value;
        }
    }

    std::uint8_t childFlags;
    read(f, childFlags);

    // remember node
    loader(tileId, node, childFlags);

    std::uint8_t mask(1 << 4);
    for (const auto &childId : children(baseTileSize, tileId)) {
        if (childFlags & mask) {
            loadMetatileTree(baseTileSize, childId, loader, f);
        }
        mask <<= 1;
    }
}

} // namespace

void loadMetatile(std::istream &f, long baseTileSize, const TileId &tileId
                  , const MetaNodeLoader &loader)
{
    using utility::binaryio::read;
    uint32_t version;
    {
        char magic[8];

        read(f, magic);
        if (std::memcmp(magic, METATILE_IO_MAGIC, sizeof(METATILE_IO_MAGIC))) {
            LOGTHROW(err1, FormatError) << "Bad metatile data magic.";
        }
        read(f, version);
        if (version > METATILE_IO_VERSION) {
            LOGTHROW(err1, FormatError)
                << "Unsupported metatile format (" << version << ").";
        }
    }

    // TODO: use version to load different versions of metatile
    loadMetatileTree(baseTileSize, tileId, loader, f);
}

namespace {

struct MetatileDef {
    TileId id;
    Lod end;

    MetatileDef(const TileId id, Lod end)
        : id(id), end(end)
    {}

    bool bottom() const { return (id.lod + 1) >= end; }
};

class Saver {
public:
    Saver(long baseTileSize, const LodLevels &metaLevels
          , const MetaNodeSaver &saver)
        : baseTileSize(baseTileSize), metaLevels(metaLevels)
        , saver(saver)
    {}

    void operator()(const TileId &foat);

private:
    void saveMetatile(const MetatileDef &tile);

    void saveMetatileTree(std::ostream &f, const MetatileDef &tile);

    long baseTileSize;
    LodLevels metaLevels;
    const MetaNodeSaver &saver;

    std::queue<MetatileDef> subtrees;
};

void Saver::operator()(const TileId &foat)
{
    subtrees.emplace(foat, deltaDown(metaLevels, foat.lod));
    while (!subtrees.empty()) {
        saveMetatile(subtrees.front());
        subtrees.pop();
    }
}

void Saver::saveMetatileTree(std::ostream &f, const MetatileDef &tile)
{
    using utility::binaryio::write;

    auto bottom(tile.bottom());

    const auto *node(saver.getNode(tile.id));
    if (!node) {
        LOGTHROW(err2, Error)
            << "Can't find metanode for tile " << tile.id;
    }

    LOG(info2) << "Dumping " << tile.id << ", " << tile.end
               << ", bottom: " << bottom
               << ", meta mode:\n" << utility::dump(*node, "    ");

    // save
    node->dump(f);

    std::uint8_t childFlags(0);
    std::uint8_t mask(1);
    auto childrenIds(children(baseTileSize, tile.id));

    for (auto &childId : childrenIds) {
        LOG(debug) << "processing child: " << childId;
        if (saver.getNode(childId)) {
            childFlags |= mask;
        }
        mask <<= 1;
    }

    if (!bottom) {
        // children are local
        childFlags |= (childFlags << 4);
    }
    write(f, childFlags);

    LOG(info2) << "    child flags: " << std::bitset<8>(childFlags);

    // either dump 4 subnodes now or remember them in the subtrees
    mask = 1;
    for (const auto &childId : childrenIds) {
        if ((childFlags & mask)) {
            if (bottom) {
                // we are at the bottom of the metatile; remember subtree
                subtrees.emplace
                    (childId, deltaDown(metaLevels, childId.lod));
            } else {
                // save subtree in this tile
                saveMetatileTree
                    (f , { childId, deltaDown(metaLevels, childId.lod) });
            }
        }
        mask <<= 1;
    }
}

void Saver::saveMetatile(const MetatileDef &tile)
{
    saver.saveTile(tile.id, [this, &tile](std::ostream &f) {
            using utility::binaryio::write;
            write(f, METATILE_IO_MAGIC);
            write(f, uint32_t(METATILE_IO_VERSION));
            saveMetatileTree(f, tile);
        });
}

} // namespace

void saveMetatile(long baseTileSize, const TileId &foat
                  , const LodLevels &metaLevels
                  , const MetaNodeSaver &saver)
{
    Saver(baseTileSize, metaLevels, saver)(foat);
}

} } // namespace vadstena::tilestorage
