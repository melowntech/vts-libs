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
 * \file vts/storage.hpp
 * \author Vaclav Blazek <vaclav.blazek@citationtech.net>
 *
 * Tile set storage access.
 */

#ifndef vtslibs_vts_storage_hpp_included_
#define vtslibs_vts_storage_hpp_included_

#include <memory>
#include <string>

#include <boost/optional.hpp>
#include <boost/noncopyable.hpp>
#include <boost/filesystem/path.hpp>

#include "utility/enum-io.hpp"
#include "utility/runnable.hpp"

#include "tileset.hpp"
#include "basetypes.hpp"
#include "glue.hpp"
#include "virtualsurface.hpp"
#include "options.hpp"
#include "storage/locking.hpp"

namespace vtslibs { namespace vts {

struct StorageProperties {
    std::string referenceFrame;
};

struct ExtraStorageProperties {
    /** Switches on/off virtual surface support.
     *  On by default.
     */
    bool virtualSurfacesEnabled;

    /** Override position.
     */
    boost::optional<registry::Position> position;

    /** ROI server(s) definition.
     */
    registry::Roi::list rois;

    /** Initial view.
     */
    registry::View view;

    /** Named views.
     */
    registry::View::map namedViews;

    /** Credits definition to include in the output.
     */
    registry::Credit::dict credits;

    /** Bound layers definition to include in the output.
     */
    registry::BoundLayer::dict boundLayers;

    /** Free layers definition to include in the output.
     */
    registry::FreeLayer::dict freeLayers;

    /** Browser core options. Opaque structure.
     */
    boost::any browserOptions;

    /** Extra bodies.
     */
    registry::Body::dict bodies;

    ExtraStorageProperties() : virtualSurfacesEnabled(true) {}
};

typedef std::set<std::string> Tags;

/** Proxy name to external URL mapping.
 */
typedef std::map<std::string, std::string> Proxy2ExternalUrl;

/** Info about stored tileset
 */
struct StoredTileset {
    /** Unique tileset identifier.
     */
    TilesetId tilesetId;

    /** Base identifier (without version).
     */
    TilesetId baseId;

    /** Version of tileset
     */
    int version;

    /** Tileset tags
     */
    Tags tags;

    /** Proxy to external URL mapping.
     */
    Proxy2ExternalUrl proxy2ExternalUrl;

    typedef std::vector<StoredTileset> list;

    typedef std::vector<const StoredTileset*> constptrlist;

    StoredTileset() : version() {}

    StoredTileset(const TilesetId &tilesetId)
        : tilesetId(tilesetId), version()
    {}
};

class TileFilter {
public:
    boost::optional<LodRange> lodRange() const { return lodRange_; }

    TileFilter& lodRange(const boost::optional<LodRange> &lodRange) {
        lodRange_ = lodRange;  return *this;
    }

private:
    boost::optional<LodRange> lodRange_;
};

class PendingGluesError : public std::runtime_error {
public:
    PendingGluesError(Glue::IdSet glues)
        : std::runtime_error("Penging glues.")
        , glues_(std::move(glues))
    {}

    // needed by old gcc
    virtual ~PendingGluesError() throw() {}

    const Glue::IdSet& glues() const { return glues_; }

private:
    Glue::IdSet glues_;
};

/** Storage interface.
 */
class Storage {
public:
    /** Opens existing storage.
     */
    Storage(const boost::filesystem::path &path, OpenMode mode
            , const StorageLocker::pointer &locker = nullptr);

    /** Creates new storage.
     */
    Storage(const boost::filesystem::path &path
            , const StorageProperties &properties
            , CreateMode mode
            , const StorageLocker::pointer &locker = nullptr);

    ~Storage();

    struct Location {
        std::string where;

        enum class Direction { below, above };
        Direction direction;

        Location(const std::string &where, Direction direction)
            : where(where), direction(direction)
        {}

        Location() : direction(Direction::below) {}
    };

    /** Add options:
     *  bumpVersion: bump version in case of ID collision
     *  textureQuality: JPEG quality of glue textures 0 means no atlas repacking
     *  filter optional: filter for input dataset
     *  dryRun: do not modify anything, simulate add
     *  tags: set of tags assigned to added tileset
     *  openOptions: options for tileset open
     *  mode: glue generation mode
     *  overwrite: allow glue overwrite
     */
    struct AddOptions : public GlueCreationOptions {
        bool bumpVersion;
        TileFilter filter;
        bool dryRun;
        boost::optional<boost::filesystem::path> tmp;
        Tags tags;
        OpenOptions openOptions;
        enum Mode { legacy, full, lazy };
        Mode mode;

        /** Check for glue existence:
         *  true: fail if glue already exist
         *  false: overwrite any existing glue
         *  indeterminate: honors existing glue
         */
        boost::tribool collisionCheck;

        /** Do not generate glue when multiple tilesets have identical tile
         *  indices
         */
        bool checkTileindexIdentity;

        AddOptions()
            : bumpVersion(false), filter(), dryRun(false)
            , mode(Mode::legacy), collisionCheck(true)
            , checkTileindexIdentity(true)
        {}
    };

    /** Adds tileset from given path to the tileset at where location.
     *  Operation fails if give tileset is already present in the stack.
     *
     *  \param tilesetPath path to source tileset
     *  \param where location in the stack where to add
     *  \param tilesetId (base) id of added tileset;
     *                   pass empty string to get ID from source
     *
     *  Tileset's own id is used if info.tilesetId is empty.
     */
    void add(const boost::filesystem::path &tilesetPath, const Location &where
             , const TilesetId &tilesetId, const AddOptions &addOptions);

    /** Removes given tileset from the storage.
     *
     * Removes all glues and virtual surfaces that reference given tileset.
     *
     *  \param tilesetIds Ids of tilesets to remove
     */
    void remove(const TilesetIdList &tilesetIds);

    void generateGlues(const TilesetId &tilesetId
                       , const AddOptions &addOptions);

    void generateGlue(const Glue::Id &glueId
                      , const AddOptions &addOptions);

    /** Creates a virtual surface from storage.
     *
     *  Operation fails if given virtual surface is already present in the stack
     *  (unless mode is Create::Mode overwrite) or any tileset referenced by
     *  virtual surface doesn't exist.
     *
     *  \param virtualSurfaceId virtual surface ID
     *  \param createOptions for create mode and static meta lod range
     */
    void createVirtualSurface( const TilesetIdSet &tilesets
                             , const CloneOptions &createOptions);

    /** Removes a virtual surface from storage.
     *
     *  \param virtualSurfaceId virtual surface ID
     */
    void removeVirtualSurface(const TilesetIdSet &tilesets);

    /** Open virtual surface tileset.
     *
     *  \param virtualSurfaceId virtual surface ID
     */
    std::tuple<VirtualSurface::Id, TileSet>
    openVirtualSurface(const TilesetIdSet &tilesets) const;

    /** Flattens content of this storage into new tileset at tilesetPath.
     *
     * \param tilesetPath path to tileset
     * \param createOptions create options
     * \param subset optional subset of tilesets
     */
    TileSet clone(const boost::filesystem::path &tilesetPath
                  , const CloneOptions &createOptions
                  , const TilesetIdSet *subset = nullptr) const;

    /** Returns list of tileset ID's of tilesets in the stacked order (bottom to
     *  top).
     */
    TilesetIdList tilesets() const;

    /** Returns list of tileset ID's of tilesets in the stacked order (bottom to
     *  top); only subset matching given set is returned.
     */
    TilesetIdList tilesets(const TilesetIdSet &subset) const;

    /** Returns list of stored tilesets in the stacked order (bottom to top);
     */
    StoredTileset::list storedTilesets() const;

    /** External URL mapping for glues.
     */
    Proxy2ExternalUrl gluesExternalUrl() const;

    /** External URL mapping for virtual surfaces.
     */
    Proxy2ExternalUrl vsExternalUrl() const;

    /** Returns list of existing glues.
     */
    Glue::map glues() const;

    /** Return list of tileset's glues (glues that have given tileset at the top
     *  of the stack).
     */
    Glue::list glues(const TilesetId &tilesetId) const;

    /** Return list of tileset's glues (glues that have given tileset at the top
     *  of the stack).
     */
    Glue::list glues(const TilesetId &tilesetId
                     , const std::function<bool(const Glue::Id&)> &filter)
        const;

    /** Return list of all pending glues.
     */
    Glue::IdSet pendingGlues() const;

    /** Return list tileset's pending glues.
     */
    Glue::IdSet pendingGlues(const TilesetId &tilesetId) const;

    /** Return list pending glues in mapconfig.
     *
     * \subset subset of tilesets to be included, use null for no limit.
     */
    Glue::IdSet pendingGlues(const TilesetIdSet *subset) const;

    /** Returns list of existing virtualSurfaces.
     */
    VirtualSurface::map virtualSurfaces() const;

    /** Return list of tileset's virtualSurfaces (virtualSurfaces that have
     *  given tileset at the top of the stack).
     */
    VirtualSurface::list virtualSurfaces(const TilesetId &tilesetId) const;

    /** Return list of tileset's virtualSurfaces (virtualSurfaces that have
     *  given tileset at the top of the stack).
     */
    VirtualSurface::list
    virtualSurfaces(const TilesetId &tilesetId
                    , const std::function<bool(const VirtualSurface::Id&)>
                    &filter) const;

    bool externallyChanged() const;

    std::time_t lastModified() const;

    vtslibs::storage::Resources resources() const;

    boost::filesystem::path path() const;

    const StorageProperties& getProperties() const;

    const registry::ReferenceFrame& referenceFrame() const;

    TileSet open(const TilesetId &tilesetId) const;

    TileSet open(const Glue &glue) const;

    boost::filesystem::path path(const TilesetId &tilesetId) const;

    boost::filesystem::path path(const Glue &glue) const;

    boost::filesystem::path path(const Glue::Id &glueId) const;

    boost::filesystem::path path(const VirtualSurface &virtualSurface) const;

    void updateTags(const TilesetId &tilesetId
                    , const Tags &add, const Tags &remove);

    void updateExternalUrl(const TilesetId &tilesetId
                           , const Proxy2ExternalUrl &add
                           , const std::vector<std::string> &remove);

    /** Generates map configuration for this storage.
     */
    MapConfig mapConfig() const;

    /** Lock stress tester. Randomly loads and writes storage.conf.
     *  Makes sense only when lock is valid.
     */
    void lockStressTest(utility::Runnable &runnable);

    /** Generates map configuration for storage at given path.
     */
    static MapConfig mapConfig(const boost::filesystem::path &path);

    /** Special mapconfig generator to get subset (i.e. storage view) map
     *  configuration.
     *
     * \param path path to storage
     * \param extra extra configuration
     * \param subset tilesets to put in output
     * \param freeLayers tilesets to be put in output as mesh-tiles free layers
     * \param prefix path prefix in all URL templates
     */
    static MapConfig mapConfig(const boost::filesystem::path &path
                               , const ExtraStorageProperties &extra
                               , const TilesetIdSet &subset
                               , const TilesetIdSet &freeLayers
                               , const boost::filesystem::path &prefix);

    /** Check for storage at given path.
     */
    static bool check(const boost::filesystem::path &path);

    static bool check(const boost::filesystem::path &path
                      , const std::string &mime);

    static void relocate(const boost::filesystem::path &root
                         , const RelocateOptions &options
                         , const std::string &prefix = "");

    static void reencode(const boost::filesystem::path &root
                         , const ReencodeOptions &options
                         , const std::string &prefix = "");

    /** Internals. Public to ease library developers' life, not to allow users
     *  to put their dirty hands in the storage's guts!
     */
    struct Detail;

private:
    std::shared_ptr<Detail> detail_;
    Detail& detail() { return *detail_; }
    const Detail& detail() const { return *detail_; }

public:
    struct Properties;
};

UTILITY_GENERATE_ENUM_IO(Storage::Location::Direction,
    ((below))
    ((above))
)

UTILITY_GENERATE_ENUM_IO(Storage::AddOptions::Mode,
    ((legacy))
    ((full))
    ((lazy))
)

template<typename CharT, typename Traits>
inline std::basic_ostream<CharT, Traits>&
operator<<(std::basic_ostream<CharT, Traits> &os, const Storage::Location &l)
{
    if (l.where.empty()) {
        if (l.direction == Storage::Location::Direction::above) {
            return os << "@BOTTOM";
        } else {
            return os << "@TOP";
        }
    }

    return os << ((l.direction == Storage::Location::Direction::below)
                  ? '-' : '+') << l.where;
}

template<typename CharT, typename Traits>
inline std::basic_istream<CharT, Traits>&
operator>>(std::basic_istream<CharT, Traits> &is, Storage::Location &l)
{
    std::string id;
    is >> id;
    if (id == "@BOTTOM") {
        l.where.erase();
        l.direction = Storage::Location::Direction::above;
        return is;
    } else if (id == "@TOP") {
        l.where.erase();
        l.direction = Storage::Location::Direction::below;
        return is;
    }

    // +/- and at least one character
    if (id.size() < 2) {
        is.setstate(std::ios::failbit);
        return is;
    }

    switch (id[0]) {
    case '-': l.direction = Storage::Location::Direction::below; break;
    case '+': l.direction = Storage::Location::Direction::above; break;
    default:
        is.setstate(std::ios::failbit);
        return is;
    }
    l.where = id.substr(1);

    return is;
}

} } // namespace vtslibs::vts

#endif // vtslibs_vts_storage_hpp_included_
