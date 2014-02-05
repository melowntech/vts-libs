#ifndef vadstena_libs_tilestorage_tileindex_hpp_included_
#define vadstena_libs_tilestorage_tileindex_hpp_included_

#include <map>

#include <boost/filesystem/path.hpp>

#include "imgproc/rastermask.hpp"

#include "../tilestorage.hpp"
#include "../entities.hpp"

namespace vadstena { namespace tilestorage {

typedef std::map<TileId, MetaNode> Metadata;

using imgproc::quadtree::RasterMask;

class TileIndex {
public:
    TileIndex() : baseTileSize_(), minLod_() {}

    TileIndex(long baseTileSize
              , Extents extents
              , LodRange lodRange
              , const TileIndex &other);

    void load(std::istream &is);
    void load(const boost::filesystem::path &path);

    void save(std::ostream &os) const;
    void save(const boost::filesystem::path &path) const;

    bool exists(const TileId &tileId) const;

    void fill(const Metadata &metadata);

    Extents extents() const;

    bool empty() const;

    Lod maxLod() const;

private:
    typedef std::vector<RasterMask> Masks;

    const RasterMask* mask(Lod lod) const;

    RasterMask* mask(Lod lod);

    void fill(Lod lod, const TileIndex &other);

    long baseTileSize_;
    Point2l origin_;
    Lod minLod_;
    Masks masks_;
};

} } // namespace vadstena::tilestorage

#endif // vadstena_libs_tilestorage_tileindex_hpp_included_
