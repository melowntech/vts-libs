#include <map>

#include "dbglog/dbglog.hpp"

#include "utility/binaryio.hpp"

#include "half/half.hpp"

#include "imgproc/rastermask/bitfield.hpp"

#include "../storage/error.hpp"

#include "./metatile.hpp"
#include "./io.hpp"

namespace fs = boost::filesystem;
namespace bin = utility::binaryio;

namespace vadstena { namespace vts {

namespace {
    const char MAGIC[2] = { 'M', 'E' };
    const std::uint16_t VERSION = 1;

    const std::size_t MIN_GEOM_BITS(2);

    struct NodeFlag {
        enum : std::uint8_t {
            geometryPresent = 0x01
            , navtilePresent = 0x02
            , internalTexturePresent = 0x04
            , coarsenessControl = 0x08

            , ulChild = 0x10
            , urChild = 0x20
            , llChild = 0x40
            , lrChild = 0x80
        };
    };
} // namespace

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

void MetaTile::set(const TileId &tileId, const MetaNode &node)
{
    auto gi(gridIndex(tileId));

    // update valid extents; NB: using plus one!
    valid_.ll(0) = std::min(valid_.ll(0), gi(0));
    valid_.ll(1) = std::min(valid_.ll(1), gi(1));
    valid_.ur(0) = std::max(valid_.ll(0), gi(0) + 1);
    valid_.ur(1) = std::max(valid_.ll(1), gi(1) + 1);

    update(valid_, gi);
    grid_[index(gi)] = node;
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

}

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
    bin::write(out, std::uint32_t(offset(0)));
    bin::write(out, std::uint32_t(offset(1)));
    auto validSize(size(valid_));
    bin::write(out, std::uint32_t(validSize.width));
    bin::write(out, std::uint32_t(validSize.height));

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
        bin::write(out, std::uint16_t(bitmap.repr()));

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

        }
    }

    // write nodes
    for (auto j(offset(1)); j < validSize.height; ++j) {
        for (auto i(offset(0)); i < validSize.width; ++i) {
            const auto &node(grid_[j * size_ + i]);
            bin::write(out, node.flags());

            // TODO: write all node's stuff here :P
        }
    }
}

void saveMetaTile(const fs::path &path, const MetaTile &meta)
{
    utility::ofstreambuf f(path.string());
    saveMetaTile(f, meta);
    f.close();
}

MetaTile loadMetaTile(std::istream &in, std::uint8_t binaryOrder
                      , const fs::path &path)
{
    (void) in;
    (void) path;
    return { {}, binaryOrder };
}

MetaTile loadMetaTile(const fs::path &path, std::uint8_t binaryOrder)
{
    utility::ifstreambuf f(path.string());
    auto meta(loadMetaTile(f, binaryOrder, path));
    f.close();
    return meta;
}

} } // namespace vadstena::vts
