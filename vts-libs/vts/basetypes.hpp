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
#ifndef vtslibs_vts_basetypes_hpp_included_
#define vtslibs_vts_basetypes_hpp_included_

#include <new>
#include <string>

#include "math/geometry_core.hpp"
#include "math/extent.hpp"

#include "utility/enum-io.hpp"

#include "../storage/lod.hpp"
#include "../storage/range.hpp"
#include "../registry.hpp"

namespace vtslibs { namespace vts {

using storage::Lod;
using storage::Range;
using storage::LodRange;

using registry::TileRange;

typedef math::Extent_<TileRange::value_type> TileSpan;

typedef registry::ReferenceFrame::Division::Node RFNode;

/** Tile identifier (index in 3D space): LOD + tile index from upper-left corner
 *  in tile grid.
 *
 *  Works as an empty wrapper around reference frame node's ID
 */
typedef RFNode::Id TileId;

struct Child : TileId {
    unsigned int index;

    Child(const TileId &tileId = TileId(), unsigned int index = 0)
        : TileId(tileId), index(index)
    {}

    Child(Lod lod, unsigned int x, unsigned int y, unsigned int index)
        : TileId(lod, x, y), index(index)
    {}
};

typedef std::array<Child, 4> Children;

/** Combination of LOD and tile range in one package.
 */
struct LodTileRange {
    Lod lod;
    TileRange range;

    LodTileRange(Lod lod = 0) : lod(lod) {}
    LodTileRange(Lod lod, const TileRange &range) : lod(lod), range(range) {}
    LodTileRange(const TileId &tileId, const math::Size2 &size)
        : lod(tileId.lod)
        , range(tileId.x, tileId.y, tileId.x + size.width - 1
                , tileId.y + size.height - 1)
    {}

    typedef std::vector<LodTileRange> list;
};

class Ranges {
public:
    Ranges() : lodRange_(LodRange::emptyRange()) {}
    Ranges(const LodRange &lodRange, const TileRange &tileRange);
    Ranges(const LodTileRange &range, Lod bottomLod);

    struct FromBottom {};
    Ranges(const LodRange &lodRange, const TileRange &tileRange
           , const FromBottom&);

    /** Returns range for given lod.
     *  Throws if lod out of range.
     */
    const TileRange& tileRange(Lod lod) const;

    /** Returns range for given lod.
     *  Returns nullptr if lod out of range.
     */
    const TileRange* tileRange(Lod lod, std::nothrow_t) const;

    /** Returns tile range at minimum lod or 0,0,0,0 if invalid.
     */
    const TileRange& tileRange() const;

    /** Returns lod range.
     */
    const LodRange& lodRange() const { return lodRange_; }

    std::vector<LodTileRange> ranges() const;

    /** Merge information from multiple ranges.
     */
    void update(const Ranges &other);

private:
    void populate(const TileRange &tileRange);

    LodRange lodRange_;
    std::vector<TileRange> tileRanges_;
};

/** Open mode
 */
enum class OpenMode {
    readOnly     //!< only getters are allowed
    , readWrite  //!< both getters and setters are allowed
};

enum class CreateMode {
    failIfExists //!< creation fails if tile set/storage already exists
    , overwrite  //!< existing tile set/storage is replace with new one
};

typedef std::string TilesetId;
typedef std::vector<TilesetId> TilesetIdList;
typedef std::set<TilesetId> TilesetIdSet;
typedef std::map<TilesetId, std::size_t> TilesetIdCounts;
typedef std::map<TilesetId, TilesetId> TilesetIdMap;

enum class FileFlavor {
    regular, raw, debug
};

// inline stuff

UTILITY_GENERATE_ENUM_IO(OpenMode,
    ((readOnly))
    ((readWrite))
)

UTILITY_GENERATE_ENUM_IO(CreateMode,
    ((failIfExists))
    ((overwrite))
)

UTILITY_GENERATE_ENUM_IO(FileFlavor,
    ((regular))
    ((raw))
    ((debug))
)

} } // namespace vtslibs::vts

#endif // vtslibs_vts_basetypes_hpp_included_
