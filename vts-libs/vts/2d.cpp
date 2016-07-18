#include "../storage/error.hpp"

#include "./2d.hpp"
#include "./qtree-rasterize.hpp"
#include "./tileop.hpp"

namespace gil = boost::gil;

namespace vadstena { namespace vts {

GrayImage mask2d(const Mesh::CoverageMask &coverageMask
                 , const std::vector<SubMesh::SurfaceReference>
                 &surfaceReferences, bool singleSourced)
{
    auto size(Mask2d::size());
    GrayImage out(size.width, size.height, gil::gray8_pixel_t(0x00), 0);

    auto outView(view(out));

    auto maskSize(Mask2d::maskSize());
    auto maskView(gil::subimage_view
                  (outView, 0, 0, maskSize.width, maskSize.height));
    rasterize(coverageMask, maskView);

    // get iterators to flag and surface rows
    auto iflag(outView.row_begin(Mask2d::flagRow));
    auto isurfaceReference(outView.row_begin(Mask2d::surfaceRow));
    // submeshes start at 1
    ++iflag;
    ++isurfaceReference;
    for (const auto &sr : surfaceReferences) {
        *iflag++ = Mask2d::Flag::submesh;
        // force single (first) source if asked to; otherwise use mapping
        *isurfaceReference++ = (singleSourced ? 1 : sr);
    }

    return out;
}

typedef TileIndex::Flag TiFlag;
constexpr TiFlag::value_type nonmaskedMask =
    (TiFlag::watertight | TiFlag::multimesh);
constexpr TiFlag::value_type nonmaskedValue =
    (TiFlag::watertight);

GrayImage meta2d(const TileIndex &tileIndex, const TileId &tileId)
{
    if (!Meta2d::isMetaId(tileId)) {
        LOGTHROW(err1, storage::NoSuchTile)
            << "Tile ID " << tileId << " is not valid for 2d metatile.";
    }

    auto size(Meta2d::size());

    GrayImage out(size.width, size.height, gil::gray8_pixel_t(0x00), 0);
    auto outView(view(out));

    if (const auto *tree = tileIndex.tree(tileId.lod)) {
        auto parentId(parent(tileId, Meta2d::binaryOrder));

        rasterize(*tree, parentId.lod, parentId.x, parentId.y
                  , outView, [&](QTree::value_type flags) -> std::uint8_t
        {
            std::uint8_t out(0);

            if (flags & TiFlag::mesh) { out |= Meta2d::Flag::geometry; }
            if (TiFlag::check(flags, nonmaskedMask, nonmaskedValue)) {
                out |= Meta2d::Flag::nonmasked;
            }

            // TODO: ophoto goes here

            if (TiFlag::isAlien(flags)) { out |= Meta2d::Flag::alien; }

            return out;
        });
    }
    return out;
}

void saveCreditTile(std::ostream &out, const CreditTile &creditTile
                    , bool inlineCredits)
{
    registry::saveCredits(out, creditTile.credits, inlineCredits);
}

CreditTile loadCreditTile(std::istream &in
                          , const boost::filesystem::path &path)
{
    CreditTile creditTile;
    registry::loadCredits(in, creditTile.credits, path);
    return creditTile;
}

void CreditTile::update(const CreditTile &other)
{
    for (const auto &credit : other.credits) {
        credits.set(credit.first, credit.second);
    }
}

void CreditTile::expand(const registry::Credit::dict &dict)
{
    registry::Credits tmpCredits;

    for (const auto &item : credits) {
        if (const auto *credit = dict.get(item.first, std::nothrow)) {
            tmpCredits.set(item.first, *credit);
        }
    }

    std::swap(credits, tmpCredits);
}

} } // vadstena::vts
