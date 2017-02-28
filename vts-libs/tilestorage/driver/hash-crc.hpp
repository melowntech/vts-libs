#ifndef vtslibs_tilestorage_driver_hash_crc_hpp_included_
#define vtslibs_tilestorage_driver_hash_crc_hpp_included_

#include "./fsbased.hpp"
#include "./factory.hpp"

namespace vtslibs { namespace tilestorage {

class HashCrcDriver : public FsBasedDriver {
public:
    HashCrcDriver(const boost::filesystem::path &root, CreateMode mode
                  , const CreateProperties &properties)
        : FsBasedDriver(root, mode, properties)
    {}

    /** Opens storage.
     */
    HashCrcDriver(const boost::filesystem::path &root, OpenMode mode
                  , const DetectionContext &context)
        : FsBasedDriver(root, mode, context)
    {}

    virtual ~HashCrcDriver() {}

    static const std::string help;

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

    virtual DriverProperties properties_impl() const UTILITY_OVERRIDE {
        return { Factory::staticType(), {} };
    }
};

} } // namespace vtslibs::tilestorage

#endif // vtslibs_tilestorage_driver_hash_crc_hpp_included_
