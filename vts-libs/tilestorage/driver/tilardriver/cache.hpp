#ifndef vadstena_libs_tilestorage_driver_tilardriver_tilarcache_hpp_included_
#define vadstena_libs_tilestorage_driver_tilardriver_tilarcache_hpp_included_

#include <set>
#include <map>

#include <boost/noncopyable.hpp>
#include <boost/optional.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/uuid/uuid.hpp>

#include "../../tilar.hpp"
#include "../../basetypes.hpp"

#include "./options.hpp"

namespace vadstena { namespace tilestorage { namespace tilardriver {

namespace fs = boost::filesystem;

class Cache : boost::noncopyable {
public:
    Cache(const fs::path &root, const Options &options, bool readOnly);

    ~Cache();

    IStream::pointer input(const TileId tileId, TileFile type);

    OStream::pointer output(const TileId tileId, TileFile type);

    /** Removes file but doesn't throw if file didn't exist.
     */
    void remove(const TileId tileId, TileFile type);

    std::size_t size(const TileId tileId, TileFile type);

    FileStat stat(const TileId tileId, TileFile type);

    Driver::Resources resources();

    void flush();
    void commit();
    void rollback();

private:
    struct Archives;

    Archives& getArchives(TileFile type);

    const fs::path &root_;
    const Options &options_;
    const bool readOnly_;

    std::unique_ptr<Archives> tiles_;
    std::unique_ptr<Archives> metatiles_;
};

inline Cache::Archives& Cache::getArchives(TileFile type)
{
    switch (type) {
    case TileFile::meta: return *metatiles_;
    case TileFile::mesh: case TileFile::atlas: return *tiles_;
    }
    throw;
}

} } } // namespace vadstena::tilestorage::tilardriver

#endif // vadstena_libs_tilestorage_driver_tilardriver_tilarcache_hpp_included_
