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
     *
     * \param contentOnly takes into account only tiles with real data.
     */
    TileIndex deriveMetaIndex(bool contentOnly = false) const;

    /** Derives metatile index from bottom to given lod
     *
     * \param contentOnly takes into account only tiles with real data.
     */
    TileIndex deriveMetaIndex(Lod upperLod, bool contentOnly = false) const;

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
