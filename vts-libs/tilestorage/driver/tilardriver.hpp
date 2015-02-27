#ifndef vadstena_libs_tilestorage_driver_tilardriver_hpp_included_
#define vadstena_libs_tilestorage_driver_tilardriver_hpp_included_

#include <set>
#include <map>

#include <boost/optional.hpp>
#include <boost/filesystem/path.hpp>

#include "../driver.hpp"
#include "./factory.hpp"
#include "./fstreams.hpp"

namespace vadstena { namespace tilestorage {

namespace fs = boost::filesystem;

class TilarDriver : public Driver {
public:
    /** Creates new storage. Existing storage is overwritten only if mode ==
     *  CreateMode::overwrite.
     */
    TilarDriver(const fs::path &root, CreateMode mode
                , const StaticProperties &properties);

    /** Opens storage.
     */
    TilarDriver(const fs::path &root, OpenMode mode);

    virtual ~TilarDriver();

    static std::string detectType_impl(const std::string &location);

    static const std::string help;

    VADSTENA_TILESTORAGE_DRIVER_FACTORY("tilar", TilarDriver);

private:
    virtual OStream::pointer output_impl(File type) UTILITY_OVERRIDE;

    virtual IStream::pointer input_impl(File type) const UTILITY_OVERRIDE;

    virtual OStream::pointer
    output_impl(const TileId tileId, TileFile type) UTILITY_OVERRIDE;

    virtual IStream::pointer
    input_impl(const TileId tileId, TileFile type) const UTILITY_OVERRIDE;

    virtual void remove_impl(const TileId tileId, TileFile type)
        UTILITY_OVERRIDE;

    virtual void begin_impl() UTILITY_OVERRIDE;

    virtual void commit_impl() UTILITY_OVERRIDE;

    virtual void rollback_impl() UTILITY_OVERRIDE;

    virtual void drop_impl() UTILITY_OVERRIDE;

    virtual void update_impl() UTILITY_OVERRIDE;

    virtual DriverProperties properties_impl() const UTILITY_OVERRIDE;

    void writeExtraFiles();

    /** Backing root.
     */
    const fs::path root_;

    /** temporary files backing root.
     */
    const fs::path tmp_;

    struct Options {
        /** Tile size at LOD=0.
         */
        long baseTileSize;

        /** Tile alignment. No tile exists that contains this point inside.
         */
        Alignment alignment;

        /** Binary order of magnitude of data stored in the individial tile
         *  archives (each archive has square grid of
         *  (2^binarySize_)*(2^binarySize_) tiles.
         *
         * This information maps directly to LOD-shift (tile space of tiles at
         * any LOD are stored in space of "super" tiles at (LOD - binarySize_)).
         */
        std::uint8_t binarySize;

        Options(const StaticProperties &properties);
        Options(const StaticProperties &properties, bool);
    };

    Options options_;
};

} } // namespace vadstena::tilestorage

#endif // vadstena_libs_tilestorage_driver_tilardriver_hpp_included_
