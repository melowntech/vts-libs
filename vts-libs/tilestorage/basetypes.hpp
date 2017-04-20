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
 * \file basertypes.hpp
 * \author Vaclav Blazek <vaclav.blazek@citationtech.net>
 *
 * TileStorage base types.
 *
 * NB: tile set is specified by simple URI: TYPE:LOCATION where:
 *     TYPE     is type of backing storage (i.e. access driver to use);
 *              defaults to "flat"
 *     LOCATION is type-specific location of storage (e.g. root directory for
 *              filesystem based backing)
 */

#ifndef vtslibs_tilestorage_basetypes_hpp_included_
#define vtslibs_tilestorage_basetypes_hpp_included_

#include <string>

#include "../storage/lod.hpp"
#include "../storage/range.hpp"

#include "math/geometry_core.hpp"

namespace vtslibs { namespace tilestorage {

using storage::Lod;
using storage::Range;
using storage::LodRange;

/** Tile set locator
 */
struct Locator {
    std::string type;     // tile set type (i.e. driver to use)
    std::string location; // location of tile set

    /** Parse locator from TYPE:LOCATION
     */
    Locator(const std::string &locator);

    Locator(const std::string &type, const std::string &location)
        : type(type), location(location)
    {}

    Locator() {}

    std::string asString() const;
};

/** Tile identifier (index in 3D space): LOD + coordinates of lower left corner.
 */
struct TileId {
    Lod lod;
    long easting;
    long northing;

    bool operator<(const TileId &tid) const;

    TileId(Lod lod = 0, long easting = 0, long northing = 0)
        : lod(lod), easting(easting), northing(northing)
    {}
};

/** Tile index in TileIndex.
 */
struct Index {
    Lod lod;
    long easting;
    long northing;

    bool operator<(const Index &index) const;

    bool operator==(const Index &index) const;
    bool operator!=(const Index &index) const { return !operator==(index); }

    Index(Lod lod = 0, long easting = 0, long northing = 0)
        : lod(lod), easting(easting), northing(northing)
    {}
};

typedef math::Size2_<long> Size2l;

typedef math::Point2_<long> Point2l;
typedef std::vector<Point2l> Points2l;

typedef Point2l Alignment;

typedef math::Extents2_<long> Extents;

typedef std::array<TileId, 4> TileIdChildren;
typedef std::array<Index, 4> IndexChildren;

/** Lod levels.
 */
struct LodLevels {
    Lod lod;      //!< reference lod
    Lod delta;    //!< lod step

    LodLevels() : lod(), delta() {}
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

// inline stuff

inline bool TileId::operator<(const TileId &tileId) const
{
    if (lod < tileId.lod) { return true; }
    else if (tileId.lod < lod) { return false; }

    if (easting < tileId.easting) { return true; }
    else if (tileId.easting < easting) { return false; }

    return northing < tileId.northing;
}

inline bool Index::operator<(const Index &index) const
{
    if (lod < index.lod) { return true; }
    else if (index.lod < lod) { return false; }

    if (easting < index.easting) { return true; }
    else if (index.easting < easting) { return false; }

    return northing < index.northing;
}

inline bool Index::operator==(const Index &index) const
{
    return ((lod == index.lod)
            && (easting == index.easting)
            && (northing == index.northing));
}

inline Locator::Locator(const std::string &locator)
{
    auto idx(locator.find(':'));
    if (idx == std::string::npos) {
        type = {};
        location = locator;
    } else {
        type = locator.substr(0, idx);
        location = locator.substr(idx + 1);
    }
}

inline std::string Locator::asString() const
{
    std::string s(type);
    s.push_back(':');
    s.append(location);
    return s;
}

inline bool operator==(const LodLevels &l, const LodLevels &r)
{
    // deltas must be same
    if (l.delta != r.delta) { return false; }
    // difference between reference lods must be divisible by delta
    if (std::abs(l.lod - r.lod) % l.delta) { return false; }
    return true;
}

inline bool operator!=(const LodLevels &l, const LodLevels &r)
{
    return !operator==(l, r);
}

inline Point2l point(const Index &index)
{
    return { index.easting, index.northing };
}

} } // namespace vtslibs::tilestorage

#endif // vtslibs_tilestorage_basetypes_hpp_included_
