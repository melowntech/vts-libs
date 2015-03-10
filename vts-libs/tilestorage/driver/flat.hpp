#ifndef vadstena_libs_tilestorage_driver_flat_hpp_included_
#define vadstena_libs_tilestorage_driver_flat_hpp_included_

#include "./fsbased.hpp"
#include "./factory.hpp"

namespace vadstena { namespace tilestorage {

class FlatDriver : public FsBasedDriver {
public:
    FlatDriver(const boost::filesystem::path &root, CreateMode mode
               , const CreateProperties &properties)
        : FsBasedDriver(root, mode, properties)
    {}

    /** Opens storage.
     */
    FlatDriver(const boost::filesystem::path &root, OpenMode mode)
        : FsBasedDriver(root, mode)
    {}

    virtual ~FlatDriver() {}

    static const std::string help;

    VADSTENA_TILESTORAGE_DRIVER_FACTORY("flat", FlatDriver);

private:
    virtual boost::filesystem::path fileDir_impl(File, const fs::path&)
        const UTILITY_OVERRIDE
    {
        return {};
    }

    virtual boost::filesystem::path
    fileDir_impl(const TileId&, TileFile, const fs::path&)
        const UTILITY_OVERRIDE
    {
        return {};
    }

    virtual DriverProperties properties_impl() const UTILITY_OVERRIDE {
        return { Factory::staticType(), {} };
    }
};

} } // namespace vadstena::tilestorage

#endif // vadstena_libs_tilestorage_driver_flat_hpp_included_
