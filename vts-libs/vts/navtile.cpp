#include "dbglog/dbglog.hpp"

#include "utility/binaryio.hpp"

#include "../storage/error.hpp"

#include "./navtile.hpp"

namespace vtslibs { namespace vts {

namespace {
    const std::string MAGIC("NT");
    const std::uint16_t VERSION = 1;

    void checkMaskSize(const NavTile::CoverageMask &mask) {
        if (mask.size() != NavTile::size()) {
            LOGTHROW(err1, storage::FormatError)
                << "Navigation coverage mask has different "
                << "dimensions than expected " << NavTile::size()
                << " (" << mask.size() << ").";
        }
    }

} // namespace

multifile::Table NavTile::readTable(std::istream &is
                                  , const boost::filesystem::path &path)
{
    return multifile::readTable(is, MAGIC, path)
        .versionAtMost(VERSION, path)
        .checkEntryCount(2, path);
}

void NavTile::serialize(std::ostream &os) const
{
    checkMaskSize(coverageMask_);

    auto table(serialize_impl(os).set(VERSION, MAGIC));

    // write mask
    auto pos(os.tellp());
    coverageMask_.dump(os);
    table.add(pos, os.tellp() - pos);

    multifile::writeTable(table, os);
}

void NavTile::serializeNavtileProper(std::ostream &os) const
{
    checkMaskSize(coverageMask_);
    serialize_impl(os).set(VERSION, MAGIC);
}

void NavTile::deserialize(const HeightRange &heightRange, std::istream &is
                        , const boost::filesystem::path &path)
{
    auto table(readTable(is, path));
    deserialize_impl(heightRange, is, path, table);

    // read mask
    is.seekg(table.entries[1].start);
    coverageMask_.load(is);
    checkMaskSize(coverageMask_);
}

void NavTile::coverageMask(const CoverageMask &mask)
{
    checkMaskSize(mask);
    coverageMask_ = mask;
}

multifile::Table RawNavTile::serialize_impl(std::ostream &os) const
{
    using utility::binaryio::write;

    // write
    multifile::Table table;
    auto pos(os.tellp());

    write(os, image_.data(), image_.size());
    pos = table.add(pos, image_.size());

    return table;
}

void RawNavTile::deserialize_impl(const HeightRange &heightRange
                                  , std::istream &is
                                  , const boost::filesystem::path &path
                                  , const multifile::Table &table)
{
    (void) path;
    using utility::binaryio::read;

    heightRange_ = heightRange;

    const auto &entry(table[imageIndex()]);

    is.seekg(entry.start);
    image_.resize(entry.size);
    read(is, image_.data(), image_.size());
}

math::Point2 NavTile::sds2px(const math::Point2 &point
                            , const math::Extents2 &extents)
{
    // localized point (zero place to upper-left corner, X grows right, Y grows
    // down)
    math::Point2 p(point(0) - extents.ll(0), extents.ur(1) - point(1));
    // navtile pixel size
    const auto s(size());
    // extents size
    const auto es(math::size(extents));

    // pixel size inside navtile
    return { ((s.width - 1) * p(0)) / es.width
            , ((s.height - 1) * p(1)) / es.height };
}

} } // namespace vtslibs::vts
