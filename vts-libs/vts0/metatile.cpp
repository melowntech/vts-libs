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
#include <queue>
#include <bitset>
#include <iostream>
#include <limits>
#include <algorithm>
#include <cfenv>

#include "dbglog/dbglog.hpp"
#include "utility/binaryio.hpp"
#include "utility/algorithm.hpp"
#include "math/geometry.hpp"
#include "math/math.hpp"
#include "half/half.hpp"

#include "../storage/error.hpp"

#include "tileop.hpp"
#include "io.hpp"
#include "metatile.hpp"

namespace half = half_float::detail;

namespace vtslibs { namespace vts0 {

namespace {
    constexpr unsigned int METATILE_IO_VERSION_ABSOLUTE_HEIGHTFIELD = 1;
    constexpr unsigned int METATILE_IO_VERSION_SCALED_HEIGHTFIELD = 2;
    constexpr unsigned int METATILE_IO_VERSION_GDS_AND_COARSENESS = 3;

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

        // check for too small range -> return minimum
        if ((max - min) < Epsilon) { return IMin; }

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

    if( version >=METATILE_IO_VERSION_GDS_AND_COARSENESS){
        utility::array::for_each(pixelSize, [&, this](const float value)
        {
            bin::write(f, half::float2half<std::round_to_nearest>(value) );
        });
        //save pixelsize gsd and coarseness as half-float
        bin::write(f, half::float2half<std::round_to_nearest>(gsd) );
        bin::write(f, half::float2half<std::round_to_nearest>(coarseness) );
    }
    else{
        utility::array::for_each(pixelSize, [&, this](const float value)
        {
            bin::write(f, value);
        });
    }

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

    if( version >=METATILE_IO_VERSION_GDS_AND_COARSENESS){
        utility::array::for_each(pixelSize, [&, this](float &value)
        {
            std::uint16_t half;
            bin::read(f, half);
            value = half::half2float(half);
        });
        std::uint16_t half;
        bin::read(f, half );
        gsd = half::half2float(half);
        bin::read(f, half );
        coarseness = half::half2float(half);
    }
    else{
        utility::array::for_each(pixelSize, [&, this](float &value)
        {
            bin::read(f, value);
        });
    }

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

const unsigned METATILE_IO_VERSION = METATILE_IO_VERSION_GDS_AND_COARSENESS;
//const unsigned METATILE_IO_VERSION = METATILE_IO_VERSION_SCALED_HEIGHTFIELD;
//const unsigned METATILE_IO_VERSION = METATILE_IO_VERSION_ABSOLUTE_HEIGHTFIELD;

void loadMetatileTree(const TileId &tileId
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
    for (const auto &childId : children(tileId)) {
        if (childFlags & lmask) {
            // node exists and is inside this metatile
            loadMetatileTree(childId, loader, notify, f, version);
        } else if ((childFlags & gmask) && notify) {
            // node exists but lives inside other metatile
            notify(childId);
        }

        lmask <<= 1;
        gmask <<= 1;
    }
}

} // namespace

void loadMetatile(std::istream &f, const TileId &tileId
                  , const MetaNodeLoader &loader, const MetaNodeNotify &notify)
{
    using utility::binaryio::read;
    uint32_t version;
    {
        char magic[8];

        read(f, magic);
        if (std::memcmp(magic, METATILE_IO_MAGIC, sizeof(METATILE_IO_MAGIC))) {
            LOGTHROW(err1, storage::FormatError) << "Bad metatile data magic.";
        }
        read(f, version);
        if (version > METATILE_IO_VERSION) {
            LOGTHROW(err1, storage::FormatError)
                << "Unsupported metatile format (" << version << ").";
        }
    }

    loadMetatileTree(tileId, loader, notify, f, version);
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
    Saver(const LodLevels &metaLevels
          , const unsigned int version
          , const MetaNodeSaver &saver)
        : metaLevels(metaLevels), version(version), saver(saver)
    {}

    void operator()(const TileId &foat);

private:
    void saveMetatile(const MetatileDef &tile);

    void saveMetatileTree(std::ostream &f, const MetatileDef &tile);

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
        LOGTHROW(err2, storage::Error)
            << "Can't find metanode for tile " << tile.id;
    }

    LOG(debug) << "Dumping " << tile.id << ", " << tile.end
               << ", bottom: " << bottom
               << ", meta mode:\n" << utility::dump(*node, "    ");

    // save
    node->dump(f, version);

    std::uint8_t childFlags(0);
    std::uint8_t mask(1);
    auto childrenIds(children(tile.id));

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

    LOG(info1) << "    child flags: " << std::bitset<8>(childFlags);

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

void saveMetatile(const TileId &foat
                  , const LodLevels &metaLevels
                  , const MetaNodeSaver &saver)
{
    Saver(metaLevels, METATILE_IO_VERSION, saver)(foat);
}

} } // namespace vtslibs::vts0
