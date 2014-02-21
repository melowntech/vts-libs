#include <iostream>

#include "dbglog/dbglog.hpp"
#include "utility/binaryio.hpp"

#include "./tileop.hpp"
#include "./metatile.hpp"

namespace vadstena { namespace tilestorage {

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

    // remember node
    loader(tileId, node);

    std::uint8_t childFlags;
    read(f, childFlags);

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

} } // namespace vadstena::tilestorage
