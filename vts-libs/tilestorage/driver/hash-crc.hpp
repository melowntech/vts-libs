#ifndef vadstena_libs_tilestorage_driver_hash_crc_hpp_included_
#define vadstena_libs_tilestorage_driver_hash_crc_hpp_included_

#include "./fsbased.hpp"
#include "./factory.hpp"

namespace vadstena { namespace tilestorage {

class HashCrcDriver : public FsBasedDriver {
public:
    HashCrcDriver(const boost::filesystem::path &root, CreateMode mode)
        : FsBasedDriver(root, mode)
    {}

    /** Opens storage.
     */
    HashCrcDriver(const boost::filesystem::path &root, OpenMode mode)
        : FsBasedDriver(root, mode)
    {}

    virtual ~HashCrcDriver() {}

    VADSTENA_TILESTORAGE_DRIVER_FACTORY("hash/crc", HashCrcDriver);

private:
    /** Non-tile files reside in root.
     */
    virtual boost::filesystem::path fileDir_impl(File, const fs::path&)
        const UTILITY_OVERRIDE
    {
        return {};
    }

    virtual boost::filesystem::path
    fileDir_impl(const TileId &tileId, TileFile type, const fs::path &name)
        const UTILITY_OVERRIDE;
};

} } // namespace vadstena::tilestorage

#endif // vadstena_libs_tilestorage_driver_hash_crc_hpp_included_
