#include <queue>
#include <bitset>
#include <iostream>
#include <limits>
#include <algorithm>

#include "dbglog/dbglog.hpp"
#include "utility/binaryio.hpp"
#include "utility/algorithm.hpp"
#include "math/geometry.hpp"
#include "math/math.hpp"

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


namespace {
    constexpr unsigned int METATILE_IO_VERSION_ABSOLUTE_HEIGHTFIELD = 1;
    constexpr unsigned int METATILE_IO_VERSION_SCALED_HEIGHTFIELD = 2;

    template <typename IntType>
    float height2Save(const float height, float min, float max)
    {
        // min-max range sanitizer
        constexpr float Epsilon(1e-10);

        // get limits of given type as floats
        constexpr float IMin(std::numeric_limits<IntType>::min());
        constexpr float IMax(std::numeric_limits<IntType>::max());

        // sanity check
        if (min > max) { std::swap(min, max); }

        // check for too small range
        if ((max - min) < Epsilon) { return 0.0; }

        // newHeight = newMinimum + oldOffsett * scale
        // scale = newRange / oldRange
        return math::clamp
            (IMin + (height - min) * ((IMax - IMin) / (max - min))
             , IMin, IMax);
    }

    template <typename IntType>
    float height2Load(const float height, float min, float max)
    {
        // get limits of given type as floats
        constexpr float IMin(std::numeric_limits<IntType>::min());
        constexpr float IMax(std::numeric_limits<IntType>::max());

        // sanity check
        if (min > max) { std::swap(min, max); }

        // newHeight = newMinimum + oldOffsett * scale
        // scale = newRange / oldRange
        return math::clamp
            (min + (height - IMin) * ((max - min) / (IMax - IMin))
             , min, max);
    }

    template <typename T, int rows, int cols>
    std::pair<float, float> minmax(const T(&data)[rows][cols])
    {
        auto r(std::minmax_element(&data[0][0], &data[rows - 1][cols]));
        return { *r.first, *r.second };
    }
} // namespace

void MetaNode::dump(std::ostream &f, const unsigned int version) const
{
    namespace bin = utility::binaryio;

    // write Z-box as floored/cieled 16bit signed integers
    bin::write(f, std::int16_t(std::floor(zmin)));
    bin::write(f, std::int16_t(std::ceil(zmax)));

    utility::array::for_each(pixelSize, [&, this](const float value)
    {
        bin::write(f, value);
    });

    // heightmax minimum/maximum was added in SCALED-HEIGHTFIELD version
    std::int16_t hmin(0), hmax(0);
    if (version >= METATILE_IO_VERSION_SCALED_HEIGHTFIELD) {
        // calculate and write minimum/maximum heigthmap values
        float hmin_, hmax_;
        std::tie(hmin_, hmax_) = minmax(heightmap);
        hmin = std::floor(hmin_);
        hmax = std::ceil(hmax_);
        bin::write(f, hmin);
        bin::write(f, hmax);
    }

    utility::array::for_each(heightmap, [&, this](float height)
    {
        if (version == METATILE_IO_VERSION_ABSOLUTE_HEIGHTFIELD) {
            // write height as is
        } else {
            // map height from izmin/izmax range to int16_t range
            height = height2Save<std::int16_t>(height, hmin, hmax);
        }
        // write rounded height as 16bit signed integer
        bin::write(f, std::int16_t(round(height)));
    });
}

void MetaNode::load(std::istream &f, const unsigned int version)
{
    namespace bin = utility::binaryio;

    std::int16_t zmin_, zmax_;
    bin::read(f, zmin_);
    bin::read(f, zmax_);
    zmin = zmin_;
    zmax = zmax_;

    utility::array::for_each(pixelSize, [&, this](float &value)
    {
        bin::read(f, value);
    });

    // heightmax minimum/maximum was added in SCALED-HEIGHTFIELD version
    std::int16_t hmin(0), hmax(0);
    if (version >= METATILE_IO_VERSION_SCALED_HEIGHTFIELD) {
        // load minimum/maximum heigthmap values
        bin::read(f, hmin);
        bin::read(f, hmax);
    }

    utility::array::for_each(heightmap, [&, this](float &height)
    {
        std::int16_t value;
        bin::read(f, value);

        if (version == METATILE_IO_VERSION_ABSOLUTE_HEIGHTFIELD) {
            // read height as is
            height = value;
        } else {
            // map height from int16_t to zmin/zmax range range
            height = height2Load<int16_t>(value, hmin, hmax);
        }
    });
}

namespace {
const char METATILE_IO_MAGIC[8] = {  'M', 'E', 'T', 'A', 'T', 'I', 'L', 'E' };

const unsigned METATILE_IO_VERSION = METATILE_IO_VERSION_SCALED_HEIGHTFIELD;
//const unsigned METATILE_IO_VERSION = METATILE_IO_VERSION_ABSOLUTE_HEIGHTFIELD;

void loadMetatileTree(long baseTileSize, const TileId &tileId
                      , const MetaNodeLoader &loader
                      , const MetaNodeNotify &notify
                      , std::istream &f
                      , const unsigned int version)
{
    using utility::binaryio::read;

    MetaNode node;
    node.load(f, version);

    std::uint8_t childFlags;
    read(f, childFlags);

    // remember node
    loader(tileId, node, childFlags);

    std::uint8_t gmask(1);
    std::uint8_t lmask(1 << 4);
    for (const auto &childId : children(baseTileSize, tileId)) {
        if (childFlags & lmask) {
            // node exists and is inside this metatile
            loadMetatileTree(baseTileSize, childId, loader, notify
                             , f, version);
        } else if ((childFlags & gmask) && notify) {
            // node exists but lives inside other metatile
            notify(childId);
        }

        lmask <<= 1;
        gmask <<= 1;
    }
}

} // namespace

void loadMetatile(std::istream &f, long baseTileSize, const TileId &tileId
                  , const MetaNodeLoader &loader, const MetaNodeNotify &notify)
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

    loadMetatileTree(baseTileSize, tileId, loader, notify, f, version);
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
          , const unsigned int version
          , const MetaNodeSaver &saver)
        : baseTileSize(baseTileSize), metaLevels(metaLevels)
        , version(version), saver(saver)
    {}

    void operator()(const TileId &foat);

private:
    void saveMetatile(const MetatileDef &tile);

    void saveMetatileTree(std::ostream &f, const MetatileDef &tile);

    long baseTileSize;
    LodLevels metaLevels;
    const unsigned int version;
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
    node->dump(f, version);

    std::uint8_t childFlags(0);
    std::uint8_t mask(1);
    auto childrenIds(children(baseTileSize, tile.id));

    for (const auto &childId : childrenIds) {
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
                    (f, { childId, deltaDown(metaLevels, childId.lod) });
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
            write(f, uint32_t(version));
            saveMetatileTree(f, tile);
        });
}

} // namespace

void saveMetatile(long baseTileSize, const TileId &foat
                  , const LodLevels &metaLevels
                  , const MetaNodeSaver &saver)
{
    Saver(baseTileSize, metaLevels, METATILE_IO_VERSION, saver)(foat);
}

} } // namespace vadstena::tilestorage
