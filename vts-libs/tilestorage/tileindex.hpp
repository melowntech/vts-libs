#ifndef vadstena_libs_tilestorage_tileindex_hpp_included_
#define vadstena_libs_tilestorage_tileindex_hpp_included_

#include <map>

#include <boost/filesystem/path.hpp>

#include "imgproc/rastermask.hpp"

#include "./tileop.hpp"

#include "../tilestorage.hpp"
#include "../entities.hpp"

namespace vadstena { namespace tilestorage {

typedef std::map<TileId, MetaNode> Metadata;

using imgproc::quadtree::RasterMask;

class TileIndex {
public:
    TileIndex(long baseTileSize = 0)
        : baseTileSize_(baseTileSize), minLod_() {}

    TileIndex(const Alignment &alignment, long baseTileSize
              , Extents extents
              , LodRange lodRange
              , const TileIndex *other = nullptr);

    void load(std::istream &is);
    void load(const boost::filesystem::path &path);

    void save(std::ostream &os) const;
    void save(const boost::filesystem::path &path) const;

    bool exists(const TileId &tileId) const;

    void fill(const Metadata &metadata);

    void fill(Lod lod, const TileIndex &other);

    void fill(const TileIndex &other);

    void intersect(Lod lod, const TileIndex &other);

    void intersect(const TileIndex &other);

    void subtract(Lod lod, const TileIndex &other);

    void subtract(const TileIndex &other);

    Extents extents() const;

    bool empty() const;

    Lod maxLod() const;

    LodRange lodRange() const;

    TileIndex& growUp();

    TileIndex& growDown();

    TileIndex& invert();

    long baseTileSize() const { return baseTileSize_; }

private:
    typedef std::vector<RasterMask> Masks;

    const RasterMask* mask(Lod lod) const;

    RasterMask* mask(Lod lod);

    long baseTileSize_;
    Point2l origin_;
    Lod minLod_;
    Masks masks_;
};


TileIndex unite(const Alignment &alignment
                , const std::vector<const TileIndex*> &tis
                , const LodRange &baseLodRange = LodRange(0, -1));

TileIndex unite(const Alignment &alignment
                , const TileIndex &l, const TileIndex &r
                , const LodRange &baseLodRange = LodRange(0, -1));

TileIndex intersect(const Alignment &alignment
                    , const TileIndex &l, const TileIndex &r
                    , const LodRange &baseLodRange = LodRange(0, -1));

TileIndex subtract(const Alignment &alignment
                   , const TileIndex &l, const TileIndex &r
                   , const LodRange &baseLodRange = LodRange(0, -1));

// inline stuff

inline bool TileIndex::empty() const
{
    return masks_.empty();
}

inline Lod TileIndex::maxLod() const
{
    return minLod_ + masks_.size() - 1;
}

inline LodRange TileIndex::lodRange() const
{
    return { minLod_, maxLod() };
}

inline const RasterMask* TileIndex::mask(Lod lod) const
{
    auto idx(lod - minLod_);
    if ((idx < 0) || (idx > int(masks_.size()))) {
        return nullptr;
    }

    // get mask
    return &masks_[idx];
}

inline RasterMask* TileIndex::mask(Lod lod)
{
    auto idx(lod - minLod_);
    if ((idx < 0) || (idx > int(masks_.size()))) {
        return nullptr;
    }

    // get mask
    return &masks_[idx];
}

inline Extents TileIndex::extents() const
{
    if (masks_.empty()) { return { origin_, origin_ }; }

    auto size(masks_.front().size());
    auto ts(tileSize(baseTileSize_, minLod_));

    return { origin_(0), origin_(1)
            , origin_(0) + ts * size.width
            , origin_(1) + ts * size.height };
}

} } // namespace vadstena::tilestorage

#endif // vadstena_libs_tilestorage_tileindex_hpp_included_
