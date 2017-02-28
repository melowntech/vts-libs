/**
 * \file vts/tilesetindex.hpp
 * \author Vaclav Blazek <vaclav.blazek@citationtech.net>
 *
 * Tile set index access.
 */

#ifndef vtslibs_vts_tileset_tilesetindex_hpp_included_
#define vtslibs_vts_tileset_tilesetindex_hpp_included_

#include <memory>

#include "../tileindex.hpp"

namespace vtslibs { namespace vts {

class Driver;

namespace tileset {

class Index {
public:
    typedef std::shared_ptr<Index> pointer;

    Index(unsigned int metaBinaryOrder = 0)
        : metaBinaryOrder_(metaBinaryOrder)
    {}

    virtual ~Index() {};

    /** Tile index (tile data presence flags)
     */
    TileIndex tileIndex;

    bool check(const TileId &tileId, TileFile type) const;

    bool real(const TileId &tileId) const;

    bool real(const TileId &tileId, bool alien) const;

    int getReference(const TileId &tileId) const;

    bool meta(const TileId &tileId) const;

    /** Checks file type and returns flags in case of match.
     */
    TileIndex::Flag::value_type
    checkAndGetFlags(const TileId &tileId, TileFile type) const;

    /** Derives whole metatile index.
     */
    TileIndex deriveMetaIndex() const;

    /** Derives metatile index from bottom to given lod
     */
    TileIndex deriveMetaIndex(Lod upperLod) const;

    unsigned int metaBinaryOrder() const { return metaBinaryOrder_; }

    /** Loads rest of data from tile index.
     *  Default implementation loads old references tree.
     */
    void loadRest(std::istream &f, const boost::filesystem::path &path);

    /** Save rest of data to tile index.
     *  Default implementation saves old references tree.
     */
    void saveRest(std::ostream &f) const;

private:
    /** Loads rest of data from tile index.
     *  Default implementation loads old references tree.
     */
    virtual void loadRest_impl(std::istream &f
                               , const boost::filesystem::path &path);

    /** Save rest of data to tile index.
     *  Default implementation saves old references tree.
     */
    virtual void saveRest_impl(std::ostream &f) const;

    unsigned int metaBinaryOrder_;
};

void loadTileSetIndex(Index &tsi, const Driver &driver);

void saveTileSetIndex(const Index &tsi, Driver &driver);

void loadTileSetIndex(Index &tsi, const boost::filesystem::path &path);

void saveTileSetIndex(const Index &tsi, const boost::filesystem::path &path);

void saveTileSetIndex(const Index &tsi, std::ostream &os);

Index::pointer loadTileSetIndex(const Driver &driver);

// inlines

inline bool Index::real(const TileId &tileId) const
{
    return tileIndex.real(tileId);
}

inline bool Index::real(const TileId &tileId, bool alien) const
{
    return tileIndex.real(tileId, alien);
}

inline void Index::loadRest(std::istream &f
                            , const boost::filesystem::path &path)
{
    return loadRest_impl(f, path);
}

inline void Index::saveRest(std::ostream &f) const
{
    return saveRest_impl(f);
}

} } } // namespace vtslibs::vts::tileset

#endif // vtslibs_vts_tileset_detail_hpp_included_
