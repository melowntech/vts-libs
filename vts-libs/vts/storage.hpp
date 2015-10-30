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

namespace vadstena { namespace vts {

struct StorageProperties {
    std::string referenceFrame;
};

struct ExtraStorageProperties {
};

typedef std::vector<std::string> TilesetIdList;

struct Glue {
    typedef TilesetIdList Id;
    Id id;
    std::string path;

    typedef std::map<Id, Glue> map;

    Glue() {}
    Glue(const Id &id, const std::string &path) : id(id), path(path) {}

    /** Returns true if glue references given tileset
     */
    bool references(const std::string &tilesetId) const;
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

    /** Adds tileset from given path to the tileset at where location.
     *  Operation fails if give tileset is already present in the stack.
     *
     *  \param tilesetPath path to source tileset
     *  \param where location in the stack where to add
     *  \param tilesetId added tileset identifier; defaults to id of tileset at
     *                   source path
     */
    void add(const boost::filesystem::path &tilesetPath, const Location &where
             , const boost::optional<std::string> tilesetId = boost::none);

    /** Removes given tileset from the storage.
     *
     *  \param tilesetIds Ids of tilesets to remove
     */
    void remove(const TilesetIdList &tilesetIds);

    /** Returns list of tileses in the stacked order (bottom to top);
     */
    TilesetIdList tilesets() const;

    /** Returns list of existing glues.
     */
    Glue::map glues() const;

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
