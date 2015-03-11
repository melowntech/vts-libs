#ifndef vadstena_libs_tilestorage_driver_tilardriver_tilarcache_hpp_included_
#define vadstena_libs_tilestorage_driver_tilardriver_tilarcache_hpp_included_

#include <set>
#include <map>

#include <boost/optional.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/uuid/uuid.hpp>

#include "../../tilar.hpp"
#include "../../basetypes.hpp"

#include "./options.hpp"

namespace vadstena { namespace tilestorage { namespace tilardriver {

namespace fs = boost::filesystem;

class Cache {
public:
    Cache(const fs::path &root, const Options &options, bool readOnly);

    IStream::pointer input(const TileId tileId, TileFile type);

    OStream::pointer output(const TileId tileId, TileFile type);

    std::size_t size(const TileId tileId, TileFile type);

    void flush();

private:
    struct Archives {
        typedef std::map<Index, Tilar> Map;

        std::string extension;
        Tilar::Options options;
        Map map;

        Archives(const std::string &extension, int filesPerTile
                 , const Options &options);

        fs::path filePath(const fs::path &root, const Index &index) const;

        void flush();
    };

    Archives& getArchives(TileFile type);

    Tilar& open(Archives &archives, const Index &archive);

    const fs::path &root_;
    const Options &options_;
    const bool readOnly_;

    Archives tiles_;
    Archives metatiles_;
};

// inlines

inline Cache::Archives& Cache::getArchives(TileFile type)
{
    switch (type) {
    case TileFile::meta: return metatiles_;
    case TileFile::mesh: case TileFile::atlas: return tiles_;
    }
    throw;
}

inline void Cache::Archives::flush()
{
    for (auto &item : map) { item.second.flush(); }
}

} } } // namespace vadstena::tilestorage::tilardriver

#endif // vadstena_libs_tilestorage_driver_tilardriver_tilarcache_hpp_included_
