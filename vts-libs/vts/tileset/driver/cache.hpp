#ifndef vadstena_libs_vts_tileset_driver_cache_hpp_included_
#define vadstena_libs_vts_tileset_driver_cache_hpp_included_

#include <set>
#include <map>

#include <boost/noncopyable.hpp>
#include <boost/optional.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/uuid/uuid.hpp>

#include "../../../storage/tilar.hpp"
#include "../../../storage/streams.hpp"
#include "../../../storage/resources.hpp"
#include "../../basetypes.hpp"

#include "./options.hpp"

namespace vadstena { namespace vts { namespace driver {

using storage::OStream;
using storage::IStream;
using storage::File;
using storage::TileFile;
using storage::FileStat;
using storage::Resources;

namespace fs = boost::filesystem;

class Cache : boost::noncopyable {
public:
    Cache(const fs::path &root, const PlainDriverOptions &options
          , bool readOnly);

    ~Cache();

    IStream::pointer input(const TileId tileId, TileFile type);

    OStream::pointer output(const TileId tileId, TileFile type);

    std::size_t size(const TileId tileId, TileFile type);

    FileStat stat(const TileId tileId, TileFile type);

    storage::Resources resources();

    void flush();

    bool readOnly() const { return readOnly_; }

private:
    struct Archives;

    Archives& getArchives(TileFile type);

    const fs::path &root_;
    const PlainDriverOptions &options_;
    const bool readOnly_;

    std::unique_ptr<Archives> tiles_;
    std::unique_ptr<Archives> metatiles_;
    std::unique_ptr<Archives> navtiles_;
};

inline Cache::Archives& Cache::getArchives(TileFile type)
{
    switch (type) {
    case TileFile::meta: return *metatiles_;
    case TileFile::mesh: case TileFile::atlas: return *tiles_;
    case TileFile::navtile: return *navtiles_;
    }
    throw;
}

} } } // namespace vadstena::vts::driver

#endif // vadstena_libs_vts_tileset_driver_cache_hpp_included_
