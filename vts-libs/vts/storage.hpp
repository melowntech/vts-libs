/**
 * \file vts/storage.hpp
 * \author Vaclav Blazek <vaclav.blazek@citationtech.net>
 *
 * Tile set storage access.
 */

#ifndef vadstena_libs_vts_storage_hpp_included_
#define vadstena_libs_vts_storage_hpp_included_

#include <memory>
#include <string>

#include <boost/optional.hpp>
#include <boost/noncopyable.hpp>
#include <boost/filesystem/path.hpp>

#include "utility/enum-io.hpp"

#include "./tileset.hpp"
#include "./basetypes.hpp"
#include "./glue.hpp"
#include "./options.hpp"

namespace vadstena { namespace vts {

struct StorageProperties {
    std::string referenceFrame;
};

struct ExtraStorageProperties {
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

    /** Browser core options. Opaque structure.
     */
    std::shared_ptr<BrowserOptions> browserOptions;

    // TODO: freeLayers

    ExtraStorageProperties() {}
};

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

    typedef std::vector<StoredTileset> list;

    StoredTileset() : version(version) {}

    StoredTileset(const TilesetId &tilesetId)
        : tilesetId(tilesetId), version(version)
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

/** Storage interface.
 */
class Storage {
public:
    /** Opens existing storage.
     */
    Storage(const boost::filesystem::path &path, OpenMode mode);

    /** Creates new storage.
     */
    Storage(const boost::filesystem::path &path
            , const StorageProperties &properties
            , CreateMode mode);

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
     */
    struct AddOptions {
        bool bumpVersion;
        int textureQuality;
        TileFilter filter;
        bool dryRun;

        AddOptions()
            : bumpVersion(false), textureQuality(0), filter()
            , dryRun(false)
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

    /** Readds existing tileset.
     *  Operation fails if given tileset is not present in the storage
     *
     *  \param tilesetId identifier of the tileset in the storage.
     */
    void readd(const TilesetId &tilesetId, const AddOptions &addOptions);

    /** Removes given tileset from the storage.
     *
     *  \param tilesetIds Ids of tilesets to remove
     */
    void remove(const TilesetIdList &tilesetIds);

    /** Flattens content of this storage into new tileset at tilesetPath.
     *
     * \param tilesetPath path to tileset
     * \param createOptions create options
     * \param subset optional subset of tilesets
     */
    TileSet clone(const boost::filesystem::path &tilesetPath
                  , const CloneOptions &createOptions
                  , const TilesetIdSet *subset = nullptr) const;

    /** Returns list of tileses in the stacked order (bottom to top);
     */
    TilesetIdList tilesets() const;

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

    bool externallyChanged() const;

    std::time_t lastModified() const;

    vadstena::storage::Resources resources() const;

    boost::filesystem::path path() const;

    const StorageProperties& getProperties() const;

    const registry::ReferenceFrame& referenceFrame() const;

    TileSet open(const TilesetId &tilesetId) const;

    TileSet open(const Glue &glue) const;

    boost::filesystem::path path(const TilesetId &tilesetId) const;

    boost::filesystem::path path(const Glue &glue) const;

    /** Generates map configuration for this storage.
     */
    MapConfig mapConfig() const;

    /** Generates map configuration for storage at given path.
     */
    static MapConfig mapConfig(const boost::filesystem::path &path);

    /** Special mapconfig generator to get subset (i.e. storage view) map
     *  configuration.
     */
    static MapConfig mapConfig(const boost::filesystem::path &path
                               , const ExtraStorageProperties &extra
                               , const TilesetIdSet &subset
                               , const boost::filesystem::path &prefix);

    /** Check for storage at given path.
     */
    static bool check(const boost::filesystem::path &path);

    static void relocate(const boost::filesystem::path &root
                         , const RelocateOptions &options
                         , const std::string &prefix = "");

    /** Internals. Public to ease library developers' life, not to allow users
     *  to put their dirty hands in the storage's guts!
     */
    struct Detail;

private:
    struct DetailDeleter { void operator()(Detail*); };
    std::unique_ptr<Detail, DetailDeleter> detail_;
    Detail& detail() { return *detail_; }
    const Detail& detail() const { return *detail_; }

public:
    struct Properties;
};

UTILITY_GENERATE_ENUM_IO(Storage::Location::Direction,
    ((below))
    ((above))
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

} } // namespace vadstena::vts

#endif // vadstena_libs_vts_storage_hpp_included_
