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
    const std::uint16_t VERSION = 2;

    const std::size_t MIN_GEOM_BITS(2);
} // namespace

boost::optional<MetaTile::point_type>
MetaTile::gridIndex(const TileId &tileId, std::nothrow_t
                    , bool checkValidity) const
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

    math::Point2_<MetaTile::size_type> p(x, y);
    if (checkValidity && !math::inside(valid_, p)) {
        return boost::none;
    }
    return p;
}

MetaTile::point_type
MetaTile::gridIndex(const TileId &tileId, bool checkValidity) const
{
    if ((origin_.lod != tileId.lod)
        || (origin_.x > tileId.x)
        || (origin_.y > tileId.y))
    {
        LOGTHROW(warn1, storage::NoSuchTile)
            << "Node " << tileId << " not inside metatile " << origin_
            << ".";
    }

    size_type x(tileId.x - origin_.x);
    size_type y(tileId.y - origin_.y);
    if ((x >= size_) || (y >= size_)) {
        LOGTHROW(warn1, storage::NoSuchTile)
            << "Node " << tileId << " not inside metatile " << origin_
            << ".";
    }

    math::Point2_<MetaTile::size_type> p(x, y);
    if (checkValidity && !inside(valid_, p)) {
        LOGTHROW(warn1, storage::NoSuchTile)
            << "Node " << tileId << " not inside metatile " << origin_
            << ".";
    }
    return p;
}

const MetaNode* MetaTile::set(const TileId &tileId, const MetaNode &node)
{
    auto gi(gridIndex(tileId, false));
    math::update(valid_, gi);
    auto *n(&grid_[index(gi)]);
    *n = node;
    return n;
}

namespace {

std::size_t geomLen(Lod lod)
{
    // calculate lenght in bits, top it to the closest byte and calculate number
    // of bytes
    return (6 * (lod + MIN_GEOM_BITS) + 7) / 8;
}

std::uint8_t nodeSize(Lod lod)
{
    return (1 // flags
            + geomLen(lod)
            + 1 // internalTextureCount/reference
            + 2 // texelSize
            + 2 // displaySize
            + 2 + 2 // navtile height range
            );
}

std::vector<std::uint8_t>
buildGeomExtents(Lod lod, const math::Extents3 &extents)
{
    struct Encoder {
        Encoder(Lod lod)
            : block(1, 0), bits(2 + lod)
            , max((1 << bits) - 1)
            , out(&block.back()), outMask(0x80)
        {}

        void operator()(double value, bool ceil = false) {
            // clamp to valid range
            value = math::clamp(value, 0.0, 1.0);

            // convert to (double) index in the lod grid, round up or down based
            // on rounding value (false down, true up)
            auto dindex(value * max);
            if (ceil) {
                dindex = std::ceil(dindex);
            } else {
                dindex = std::floor(dindex);
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
        std::uint32_t max;
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

void parseGeomExtents(Lod lod, math::Extents3 &extents
                      , const std::vector<std::uint8_t> &block)
{
    struct Decoder {
        Decoder(Lod lod, const std::vector<std::uint8_t> &block)
            : block(block), bits(2 + lod)
            , max((1 << bits) - 1)
            , in(block.begin()), inMask(0x80)
        {}

        double operator()() {
            std::uint32_t index(0);

            for (std::uint32_t bm(1 << (bits - 1)); bm; bm >>= 1) {
                if (pop()) { index |= bm; }
            }

            return index / max;
        }

        bool pop() {
            if (!inMask) {
                ++in;
                inMask = 0x80;
            }

            auto value(*in & inMask);
            inMask >>= 1;
            return value;
        }

        const std::vector<std::uint8_t> &block;
        std::uint8_t bits;
        double max;
        std::vector<std::uint8_t>::const_iterator in;
        std::uint32_t inMask;
    };

    Decoder decoder(lod, block);

    extents.ll(0) = decoder();
    extents.ur(0) = decoder();
    extents.ll(1) = decoder();
    extents.ur(1) = decoder();
    extents.ll(2) = decoder();
    extents.ur(2) = decoder();
}

struct MetaTileFlag {
    typedef std::uint8_t value_type;

    enum : value_type {
        alienPlane = 0x01
    };

    typedef std::pair<value_type, MetaNode::Flag::value_type> FlagMapping;

    static std::initializer_list<FlagMapping> flagMapping;

    static value_type extract(const MetaNode &node) {
        value_type flags(0);
        for (const auto &mapping : flagMapping) {
            if (node.flags() & mapping.second) {
                flags |= mapping.first;
            }
        }
        return flags;
    }
};

std::initializer_list<MetaTileFlag::FlagMapping> MetaTileFlag::flagMapping = {
    { MetaTileFlag::alienPlane, MetaNode::Flag::alien }
};

std::uint16_t loadVersion(std::istream &in, const fs::path &path)
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
    return version;
}

} // namespace

inline void MetaNode::save(std::ostream &out, Lod lod) const
{
    bin::write(out, std::uint8_t(flags_));

    bin::write(out, buildGeomExtents(lod, extents));

    bin::write(out, (geometry()
                     ? std::uint8_t(internalTextureCount_)
                     : std::uint8_t(reference_)));

    // limit texel size to fit inside half float
    // TODO: make better
    auto ts((texelSize > 65000.0) ? 65000.0 : texelSize);
    bin::write(out, std::uint16_t
               (half::float2half<std::round_to_nearest>(ts)));
    bin::write(out, std::uint16_t(displaySize));

    bin::write(out, std::int16_t(heightRange.min));
    bin::write(out, std::int16_t(heightRange.max));
}

void MetaTile::save(std::ostream &out) const
{
    // accumulate extra flags
    MetaTileFlag::value_type flags(0);
    for_each([&](const TileId&, const MetaNode &node)
    {
        flags |= MetaTileFlag::extract(node);
    });

    bin::write(out, MAGIC);

    if (flags) {
        bin::write(out, VERSION);
    } else {
        // VERSION=1: no metatile flags -> write old format
        bin::write(out, std::uint16_t(1));
    }

    // tile id information
    bin::write(out, std::uint8_t(origin_.lod));
    bin::write(out, std::uint32_t(origin_.x));
    bin::write(out, std::uint32_t(origin_.y));

    // offset and dimensions of saved grid
    size2_type validSize;
    if (valid(valid_)) {
        auto offset(valid_.ll);
        bin::write(out, std::uint16_t(offset(0)));
        bin::write(out, std::uint16_t(offset(1)));
        validSize = math::size(valid_);
        ++validSize.width;
        ++validSize.height;
    } else {
        bin::write(out, std::uint16_t(0));
        bin::write(out, std::uint16_t(0));
    }

    bin::write(out, std::uint16_t(validSize.width));
    bin::write(out, std::uint16_t(validSize.height));

    // collect all credits
    // mapping between credit id and all nodes having it
    typedef std::map<std::uint16_t, std::vector<size_type> > CMap;
    CMap credits;
    {
        size_type idx(0);
        for (auto j(valid_.ll(1)); j <= valid_.ur(1); ++j) {
            for (auto i(valid_.ll(0)); i <= valid_.ur(0); ++i, ++idx) {
                const auto &node(grid_[j * size_ + i]);
                for (const auto cid : node.credits()) {
                    // TODO make it faster?
                    credits[cid].push_back(idx);
                }
            }
        }
    }

    if (flags) {
        // store flags
        bin::write(out, std::uint8_t(flags));
        bin::write(out, std::uint8_t(credits.size()));
    } else {
        // VERSION=1: write node size
        bin::write(out, nodeSize(origin_.lod));
        // VERSION=1: write node size
        if (credits.empty() || math::empty(validSize)) {
            // no credits -> credit block size is irrelevant
            bin::write(out, std::uint8_t(0));
            bin::write(out, std::uint16_t(0));
        } else {
            // write credit count
            bin::write(out, std::uint8_t(credits.size()));

            // create credit plane bitmap
            imgproc::bitfield::RasterMask
                bitmap(validSize.width, validSize.height);
            bin::write(out, std::uint16_t(bitmap.byteCount()));
        }
    }

    // store flag planes (if any)
    if (flags) {
        imgproc::bitfield::RasterMask bitmap;

        // save flag planes
        for (const auto &mapping : MetaTileFlag::flagMapping) {
            if (!(mapping.first & flags)) { continue; }

            // create flag plane
            bitmap.create(validSize.width, validSize.height
                          , imgproc::bitfield::RasterMask::EMPTY);

            // set flags
            for (unsigned int j(valid_.ll(1)), jj(0); j <= valid_.ur(1);
                 ++j, ++jj)
            {
                for (unsigned int i(valid_.ll(0)), ii(0); i <= valid_.ur(0);
                     ++i, ++ii)
                {
                    auto &node(grid_[j * size_ + i]);
                    if (node.flags() & mapping.second) {
                        bitmap.set(ii, jj);
                    }
                }
            }

            // write flag plane
            bitmap.writeData(out);
        }
    }

    if (!credits.empty() && !math::empty(validSize)) {
        // create credit plane bitmap
        imgproc::bitfield::RasterMask
            bitmap(validSize.width, validSize.height);

        // write credits
        for (const auto &credit : credits) {
            // creditId
            bin::write(out, std::uint16_t(credit.first));

            // fill bitmap
            bitmap.create(validSize.width, validSize.height
                          , imgproc::bitfield::RasterMask::EMPTY);
            for (auto idx : credit.second) {
                bitmap.set(idx % validSize.width, idx / validSize.width);
            }

            // write out bitmap
            bitmap.writeData(out);
        }
    }

    // write nodes if any
    if (!valid(valid_)) { return; }

    for (auto j(valid_.ll(1)); j <= valid_.ur(1); ++j) {
        for (auto i(valid_.ll(0)); i <= valid_.ur(0); ++i) {
            grid_[j * size_ + i].save(out, origin_.lod);
        }
    }
}

void saveMetaTile(const fs::path &path, const MetaTile &meta)
{
    utility::ofstreambuf f(path.string());
    meta.save(f);
    f.close();
}

inline void MetaNode::load(std::istream &in, Lod lod)
{
    // NB: flags are accumulated because they can be pre-initialized from
    // another source
    std::uint8_t f;
    bin::read(in, f);
    flags_ |= f;

    std::vector<std::uint8_t> geomExtents(geomLen(lod));
    bin::read(in, geomExtents);
    parseGeomExtents(lod, extents, geomExtents);

    std::uint8_t u8;
    std::uint16_t u16;
    std::int16_t i16;

    bin::read(in, u8);
    (geometry() ? internalTextureCount_ : reference_) = u8;

    bin::read(in, u16); texelSize = half::half2float(u16);
    bin::read(in, u16); displaySize = u16;

    bin::read(in, i16); heightRange.min = i16;
    bin::read(in, i16); heightRange.max = i16;
}

void MetaTile::load(std::istream &in, const fs::path &path)
{
    auto version(loadVersion(in, path));

    std::uint8_t u8;
    std::uint16_t u16;
    std::uint32_t u32;

    // tile id information
    bin::read(in, u8); origin_.lod = u8;
    bin::read(in, u32); origin_.x = u32;
    bin::read(in, u32); origin_.y = u32;

    // offset and dimensions of saved grid
    bin::read(in, u16); valid_.ll(0) = u16;
    bin::read(in, u16); valid_.ll(1) = u16;

    size2_type validSize;
    bin::read(in, u16); validSize.width = u16;
    bin::read(in, u16); validSize.height = u16;
    if (!math::empty(validSize)) {
        // non-empty -> just remove one
        valid_.ur(0) = valid_.ll(0) + validSize.width - 1;
        valid_.ur(1) = valid_.ll(1) + validSize.height - 1;
    } else {
        // empty -> invalid
        valid_ = extents_type(math::InvalidExtents{});
    }

    std::uint8_t flags(0);
    if (version < 2) {
        // node size (unused)
        bin::read(in, u8);
    } else {
        // flags
        bin::read(in, flags);
    }

    // credit count
    std::uint8_t creditCount;
    bin::read(in, creditCount);

    if (version < 2) {
        // read credit block size (unused)
        bin::read(in, u16);
    }

    // load flags
    if (flags) {
        for (const auto &mapping : MetaTileFlag::flagMapping) {
            if (!(mapping.first & flags)) { continue; }

            imgproc::bitfield::RasterMask
                bitmap(validSize.width, validSize.height);

            // read in bitmap
            bitmap.readData(in);

            // process whole bitmap and update credits of all nodes
            for (unsigned int j(valid_.ll(1)), jj(0); j <= valid_.ur(1);
                 ++j, ++jj)
            {
                for (unsigned int i(valid_.ll(0)), ii(0); i <= valid_.ur(0);
                     ++i, ++ii)
                {
                    auto &node(grid_[j * size_ + i]);

                    // set flag
                    if (bitmap.get(ii, jj)) {
                        node.update(mapping.second);
                    }
                }
            }
        }
    }

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
            for (unsigned int j(valid_.ll(1)), jj(0); j <= valid_.ur(1);
                 ++j, ++jj)
            {
                for (unsigned int i(valid_.ll(0)), ii(0); i <= valid_.ur(0);
                     ++i, ++ii)
                {
                    auto &node(grid_[j * size_ + i]);

                    if (bitmap.get(ii, jj)) {
                        node.addCredit(creditId);
                    }
                }
            }
        }
    }

    // read rest of nodes if any
    if (!valid(valid_)) { return; }

    for (auto j(valid_.ll(1)); j <= valid_.ur(1); ++j) {
        for (auto i(valid_.ll(0)); i <= valid_.ur(0); ++i) {
            grid_[j * size_ + i].load(in, origin_.lod);
        }
    }

}

MetaTile loadMetaTile(std::istream &in
                      , std::uint8_t binaryOrder
                      , const boost::filesystem::path &path)
{
    MetaTile meta({}, binaryOrder);
    try {
        meta.load(in, path);
    } catch (const std::exception &e) {
        LOGTHROW(err1, storage::BadFileFormat)
            << "Error loading metatile from file " << path
            << ": " << e.what()
            << "; state=" << utility::StreamState(in) << ".";
    }
    return meta;
}

MetaTile loadMetaTile(const fs::path &path, std::uint8_t binaryOrder)
{
    utility::ifstreambuf f(path.string());
    auto meta(loadMetaTile(f, binaryOrder, path));
    f.close();
    return meta;
}

MetaTile::pointer loadMetaTile(std::istream *in
                               , std::uint8_t binaryOrder
                               , const boost::filesystem::path &path)
{
    auto meta(std::make_shared<MetaTile>(TileId(), binaryOrder));
    try {
        meta->load(*in, path);
    } catch (const std::exception &e) {
        LOGTHROW(err1, storage::BadFileFormat)
            << "Error loading metatile from file " << path
            << ": " << e.what()
            << "; state=" << utility::StreamState(*in) << ".";
    }
    return meta;
}

MetaTile::pointer loadMetaTile(const fs::path *path, std::uint8_t binaryOrder)
{
    utility::ifstreambuf f(path->string());
    auto meta(loadMetaTile(&f, binaryOrder, *path));
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

void MetaNode::setChildFromId(Flag::value_type &flags, const TileId &tileId
                              , bool value)
{
    Flag::value_type flag(Flag::ulChild << child(tileId));
    if (value) {
        flags |= flag;
    } else {
        flags &= ~flag;
    }
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

MetaNode& MetaNode::internalTextureCount(std::size_t value)
{
    if (!geometry()) {
        LOGTHROW(err1, storage::Error)
            << "Cannot set internal texture count in tile wihtout geometry.";
    }
    const std::size_t limit
        (std::numeric_limits<decltype(internalTextureCount_)>::max());
    internalTextureCount_ = std::min(value, limit);
    return *this;
}

MetaNode& MetaNode::reference(std::size_t value)
{
    if (geometry()) {
        LOGTHROW(err1, storage::Error)
            << "Cannot set reference in tile with geometry.";
    }

    const std::size_t limit
        (std::numeric_limits<decltype(internalTextureCount_)>::max());
    if (value > limit) {
        LOGTHROW(err1, storage::Error)
            << "Invalid reference " << value << ", allowed maximum is "
            << limit << ".";
    }

    internalTextureCount_ = value;
    return *this;
}

MetaTile::extents_type MetaTile::validExtents() const
{
    return extents_type(origin_.x + valid_.ll(0)
                        , origin_.y + valid_.ll(1)
                        , origin_.x + valid_.ur(0)
                        , origin_.y + valid_.ur(1));
}

void MetaTile::update(const MetaTile &in, bool alien)
{
    // sanity check
    if ((origin_ != in.origin_) || (binaryOrder_ != in.binaryOrder_)) {
        LOGTHROW(err1, storage::Error)
            << "Incompatible metatiles.";
    }

    for (auto j(in.valid_.ll(1)); j <= in.valid_.ur(1); ++j) {
        for (auto i(in.valid_.ll(0)); i <= in.valid_.ur(0); ++i) {
            // first, skip real output tiles
            auto idx(j * in.size_ + i);
            auto &outn(grid_[idx]);

            // valid output -> nothing to do
            if (outn.real()) { continue; }

            // get input
            const auto &inn(in.grid_[idx]);

            // update valid extents
            math::update(valid_, point_type(i, j));

            if (inn.real(alien)) {
                // found new real/alien tile, copy node
                outn = inn;
                // reset children flags
                outn.childFlags(MetaNode::Flag::none);

                // reset alien flag
                outn.alien(false);
                continue;
            }

            // both are virtual nodes:
            // update extents
            outn.mergeExtents(inn);
        }
    }
}

void loadCreditsFromMetaTile(std::istream &in, registry::IdSet &credits
                             , const boost::filesystem::path &path)
{
    try {
        auto version(loadVersion(in, path));

        std::uint8_t u8;
        std::uint16_t u16;
        std::uint32_t u32;


        // tile id information: ignore
        bin::read(in, u8);
        bin::read(in, u32);
        bin::read(in, u32);

        // offset: ignore
        bin::read(in, u16);
        bin::read(in, u16);

        // size: this we need to know
        math::Size2 size;
        bin::read(in, u16); size.width = u16;
        bin::read(in, u16); size.height = u16;

        // skip empty metatile
        if (math::empty(size)) { return; }

        // read flags -- we need them
        std::uint8_t flags(0);
        if (version < 2) {
            // node size (unused)
            bin::read(in, u8);
        } else {
            // flags
            bin::read(in, flags);
        }

        // credit count: we need them
        std::uint8_t creditCount;
        bin::read(in, creditCount);

        // no credit planes? fine
        if (!creditCount) { return; }

        // size of all bitmap planes (flags and credits)
        const auto planeSize(imgproc::bitfield::RasterMask::byteCount(size));
        auto skipPlane([&]()
        {
            // seek plane size in advance
            in.seekg(planeSize, std::ios_base::cur);
        });

        if (version < 2) {
            // read credit block size (unused) (we have calculated it from size)
            bin::read(in, u16);
        }

        if (flags) {
            for (const auto &mapping : MetaTileFlag::flagMapping) {
                if ((mapping.first & flags)) { skipPlane(); }
            }
        }

        // and finally process credits
        while (creditCount--) {
            // load credit, rememebr
            std::uint16_t creditId;
            bin::read(in, creditId);
            credits.insert(creditId);
            // and skip plane (only if not last)
            if (creditCount) { skipPlane(); }
        }

        // done!

    } catch (const std::exception &e) {
        LOGTHROW(err1, storage::BadFileFormat)
            << "Error loading metatile from file " << path
            << ": " << e.what()
            << "; state=" << utility::StreamState(in) << ".";
    }
}

} } // namespace vadstena::vts
