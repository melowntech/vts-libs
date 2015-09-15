#include <map>

#include "dbglog/dbglog.hpp"

#include "utility/binaryio.hpp"
#include "math/math.hpp"

#include "half/half.hpp"

#include "imgproc/rastermask/bitfield.hpp"

#include "../storage/error.hpp"

#include "./metatile.hpp"
#include "./io.hpp"
#include "./tileop.hpp"

namespace fs = boost::filesystem;
namespace bin = utility::binaryio;
namespace half = half_float::detail;

namespace vadstena { namespace vts {

namespace {
    const char MAGIC[2] = { 'M', 'T' };
    const std::uint16_t VERSION = 1;

    const std::size_t MIN_GEOM_BITS(2);

    struct NodeFlag { enum : std::uint8_t {
        geometryPresent = 0x01
        , navtilePresent = 0x02
        , internalTexturePresent = 0x04
        , coarsenessControl = 0x08

        , ulChild = 0x10
        , urChild = 0x20
        , llChild = 0x40
        , lrChild = 0x80
    }; };
} // namespace

boost::optional<math::Point2_<MetaTile::size_type> >
MetaTile::gridIndex(const TileId &tileId, std::nothrow_t) const
{
    if ((origin_.lod != tileId.lod)
        || (origin_.x > tileId.x)
        || (origin_.y > tileId.y))
    {
        return boost::none;
    }

    size_type x(tileId.x - origin_.x);
    size_type y(tileId.y - origin_.y);
    if ((x >= size_) || (y >= size_)) {
        return boost::none;
    }
    return { x, y };
}

math::Point2_<MetaTile::size_type> MetaTile::gridIndex(const TileId &tileId)
    const
{
    if ((origin_.lod != tileId.lod)
        || (origin_.x > tileId.x)
        || (origin_.y > tileId.y))
    {
        LOGTHROW(err1, storage::NoSuchTile)
            << "Node " << tileId << " not inside metatile " << origin_
            << ".";
    }

    size_type x(tileId.x - origin_.x);
    size_type y(tileId.y - origin_.y);
    if ((x >= size_) || (y >= size_)) {
        LOGTHROW(err1, storage::NoSuchTile)
            << "Node " << tileId << " not inside metatile " << origin_
            << ".";
    }
    return { x, y };
}

const MetaNode* MetaTile::set(const TileId &tileId, const MetaNode &node)
{
    auto gi(gridIndex(tileId));

    // update valid extents; NB: using plus one!
    valid_.ll(0) = std::min(valid_.ll(0), gi(0));
    valid_.ll(1) = std::min(valid_.ll(1), gi(1));
    valid_.ur(0) = std::max(valid_.ll(0), gi(0) + 1);
    valid_.ur(1) = std::max(valid_.ll(1), gi(1) + 1);

    math::update(valid_, gi);

    auto *n(&grid_[index(gi)]);
    *n = node;
    return n;
}

namespace {

std::uint8_t geomLen(Lod lod)
{
    // calculate lenght in bits, top it to the closest byte and calculate number
    // of bytes
    return (6 * (lod + MIN_GEOM_BITS) + 7) / 8;
}

std::uint8_t nodeSize(Lod lod)
{
    return (1 // flags
            + geomLen(lod)
            + 2 // displaySize/meshArea
            + 2 // textureArea
            + 2 + 2 // navtile height range
            );
}

std::vector<std::uint8_t>
buildGeomExtents(Lod lod, const math::Extents3 &extents)
{
    struct Encoder {
        Encoder(Lod lod)
            : block(1, 0), bits(2 + lod)
            , count(1 << bits)
            , out(&block.back()), outMask(0x80)
        {}

        void operator()(double value, bool rounding = false) {
            // clamp to valid range
            value = math::clamp(value, 0.0, 1.0);

            // convert to (double) index in the lod grid, round up or down based
            // on rounding value (false down, true up)
            auto dindex(value * count);
            if (rounding) {
                dindex = std::floor(dindex);
            } else {
                dindex = std::ceil(dindex);
            }

            // convert to integer
            std::uint32_t index(dindex);

            for (std::uint32_t bm(1 << (bits - 1)); bm; bm >>= 1) {
                push(index & bm);
            }
        }

        void push(bool value) {
            if (!outMask) {
                block.push_back(0x00);
                out = &block.back();
                outMask = 0x80;
            }

            if (value) { *out |= outMask; }
            outMask >>= 1;
        }

        std::vector<std::uint8_t> block;
        std::uint8_t bits;
        std::uint32_t count;
        std::uint8_t *out;
        std::uint32_t outMask;
    };

    Encoder encoder(lod);

    encoder(extents.ll(0));
    encoder(extents.ur(0), true);
    encoder(extents.ll(1));
    encoder(extents.ur(1), true);
    encoder(extents.ll(2));
    encoder(extents.ur(2), true);

    return encoder.block;
}

void parseGeomExtents(Lod lod, const math::Extents3 &extents
                      , std::vector<std::uint8_t> &block)
{
    (void) lod;
    (void) extents;
    (void) block;
}

} // namespace

void MetaTile::save(std::ostream &out) const
{
    bin::write(out, MAGIC);
    bin::write(out, VERSION);

    // tile id information
    bin::write(out, std::uint8_t(origin_.lod));
    bin::write(out, std::uint32_t(origin_.x));
    bin::write(out, std::uint32_t(origin_.y));

    // offset and dimensions of saved grid
    auto offset(valid_.ll);
    bin::write(out, std::uint16_t(offset(0)));
    bin::write(out, std::uint16_t(offset(1)));
    auto validSize(size(valid_));
    bin::write(out, std::uint16_t(validSize.width));
    bin::write(out, std::uint16_t(validSize.height));

    bin::write(out, nodeSize(origin_.lod));

    // collect all credits
    // mapping between credit id and all nodes having it
    std::map<std::uint16_t, std::vector<size_type> > credits;
    {
        size_type idx(0);
        for (const auto &node : grid_) {
            for (const auto cid : node.credits) {
                // TODO make it faster?
                credits[cid].push_back(idx);
            }
            ++idx;
        }
    }

    // write credit count
    bin::write(out, std::uint16_t(credits.size()));

    if (credits.empty()) {
        // no credits -> credit block size is irrelevant
        bin::write(out, std::uint16_t(0));
    } else {
        imgproc::bitfield::RasterMask
            bitmap(validSize.width, validSize.height);

        // write credit block size
        bin::write(out, std::uint16_t(bitmap.byteCount()));

        // write credits
        for (const auto &credit : credits) {
            // creditId
            bin::write(out, std::uint16_t(credit.first));

            // fill bitmap
            bitmap.create(validSize.width, validSize.height
                          , imgproc::bitfield::RasterMask::EMPTY);
            for (auto idx : credit.second) {
                bitmap.set(idx / validSize.width, idx % validSize.width);
            }

            // write out bitmap
            bitmap.writeData(out);
        }
    }

    // write nodes
    for (auto j(valid_.ll(1)); j < valid_.ur(1); ++j) {
        for (auto i(valid_.ll(0)); i < valid_.ur(0); ++i) {
            const auto &node(grid_[j * size_ + i]);
            bin::write(out, std::uint8_t(node.flags()));

            bin::write(out, buildGeomExtents(origin_.lod, node.extents));

            switch (node.cc()) {
            case MetaNode::CoarsenessControl::texelSize:
                bin::write(out, half::float2half<std::round_to_nearest>
                           (node.meshArea));
                if (node.internalTexture()) {
                    bin::write(out, half::float2half<std::round_to_nearest>
                               (node.textureArea));
                }
                break;

            case MetaNode::CoarsenessControl::displaySize:
                bin::write(out, std::uint16_t(node.displaySize));
                break;
            }

            bin::write(out, std::int16_t(node.heightRange.min));
            bin::write(out, std::int16_t(node.heightRange.max));
        }
    }
}

void saveMetaTile(const fs::path &path, const MetaTile &meta)
{
    utility::ofstreambuf f(path.string());
    meta.save(f);
    f.close();
}

void MetaTile::load(std::istream &in, const fs::path &path)
{
    char magic[sizeof(MAGIC)];
    std::uint16_t version;

    bin::read(in, magic);
    bin::read(in, version);

    if (std::memcmp(magic, MAGIC, sizeof(MAGIC))) {
        LOGTHROW(err1, storage::BadFileFormat)
            << "File " << path << " is not a VTS metatile file.";
    }
    if (version > VERSION) {
        LOGTHROW(err1, storage::VersionError)
            << "File " << path
            << " has unsupported version (" << version << ").";
    }

    std::uint8_t u8;
    std::uint16_t u16;
    std::int16_t i16;
    std::uint32_t u32;

    // tile id information
    bin::read(in, u8); origin_.lod = u8;
    bin::read(in, u32); origin_.x = u32;
    bin::read(in, u32); origin_.y = u32;

    // offset and dimensions of saved grid
    bin::read(in, u16); valid_.ll(0) = u16;
    bin::read(in, u16); valid_.ll(1) = u16;

    math::Size2_<size_type> validSize;
    bin::read(in, u16); validSize.width = u16;
    bin::read(in, u16); validSize.height = u16;
    valid_.ur(0) = valid_.ll(0) + validSize.width;
    valid_.ur(1) = valid_.ll(1) + validSize.height;

    // node size (unused)
    bin::read(in, u16);

    // credit count
    std::uint16_t creditCount;
    bin::read(in, creditCount);

    // read credit block size (unused)
    bin::read(in, u16);

    if (creditCount) {
        imgproc::bitfield::RasterMask
            bitmap(validSize.width, validSize.height);

        while (creditCount--) {
            // read credit ID
            std::uint16_t creditId;
            bin::read(in, creditId);

            // read in bitmap
            bitmap.readData(in);

            // process whole bitmap and update credits of all nodes

            for (std::uint32_t j(valid_.ll(1)), jj(0); j < valid_.ur(1);
                 ++j, ++jj)
            {
                for (std::uint32_t i(valid_.ll(0)), ii(0); i < valid_.ur(0);
                     ++i, ++ii)
                {
                    auto &node(grid_[j * size_ + i]);

                    if (bitmap.get(ii, jj)) {
                        node.credits.push_back(creditId);
                    }
                }
            }
        }
    }

    // read rest of nodes
    for (auto j(valid_.ll(1)); j < valid_.ur(1); ++j) {
        for (auto i(valid_.ll(0)); i < valid_.ur(0); ++i) {
            auto &node(grid_[j * size_ + i]);
            std::uint8_t flags;
            bin::read(in, flags);
            node.flags(flags);

            std::vector<std::uint8_t> geomExtents(geomLen(origin_.lod), 0x00);
            bin::read(in, geomExtents);
            parseGeomExtents(origin_.lod, node.extents, geomExtents);

            switch (node.cc()) {
            case MetaNode::CoarsenessControl::texelSize:
                bin::read(in, u16);
                node.meshArea = half::half2float(u16);
                if (node.internalTexture()) {
                    bin::read(in, u16);
                    node.textureArea = half::half2float(u16);
                }
                break;

            case MetaNode::CoarsenessControl::displaySize:
                bin::read(in, u16); node.displaySize = u16;
                break;
            }

            bin::read(in, i16); node.heightRange.min = u16;
            bin::read(in, i16); node.heightRange.max = u16;
        }
    }

}

MetaTile loadMetaTile(std::istream &in
                      , std::uint8_t binaryOrder
                      , const boost::filesystem::path &path)
{
    MetaTile meta({}, binaryOrder);
    meta.load(in, path);
    return meta;
}

MetaTile loadMetaTile(const fs::path &path, std::uint8_t binaryOrder)
{
    utility::ifstreambuf f(path.string());
    auto meta(loadMetaTile(f, binaryOrder, path));
    f.close();
    return meta;
}

void MetaNode::update(const MetaNode &other)
{
    auto cf(childFlags());
    *this = other;
    childFlags(cf);
}

void MetaTile::update(const TileId &tileId, const MetaNode &mn)
{
    grid_[index(tileId)].update(mn);
}

MetaNode& MetaNode::setChildFromId(const TileId &tileId, bool value)
{
    return set(Flag::ulChild << child(tileId), value);
}

MetaNode& MetaNode::mergeExtents(const MetaNode &other)
{
    if (other.extents.ll == other.extents.ur) {
        // nothing to do
        return *this;
    }

    if (extents.ll == extents.ur) {
        // use other's extents
        extents = other.extents;
        return *this;
    }

    // merge
    extents = unite(extents, other.extents);
    return *this;
}

} } // namespace vadstena::vts
