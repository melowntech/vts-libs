#ifndef vadstena_libs_tilestorage_driver_tar_hpp_included_
#define vadstena_libs_tilestorage_driver_tar_hpp_included_

#include <set>
#include <map>

#include <boost/optional.hpp>
#include <boost/filesystem/path.hpp>

#include "utility/tar.hpp"

#include "./ro-base.hpp"
#include "./factory.hpp"

namespace vadstena { namespace tilestorage {

namespace fs = boost::filesystem;

class TarDriver : public ReadOnlyDriver {
public:
    TarDriver(const fs::path&, CreateMode mode
              , const CreateProperties &properties)
        : ReadOnlyDriver(mode, properties) {}

    /** Opens storage.
     */
    TarDriver(const fs::path &root, OpenMode mode);

    virtual ~TarDriver();

    static const std::string help;

    VADSTENA_TILESTORAGE_DRIVER_FACTORY("tar", TarDriver);

    struct Record {
        std::string path;
        std::size_t block;
        std::size_t size;
        std::time_t time;

        Record(std::string path = "", std::size_t block = 0
               , std::size_t size = 0, std::time_t time = 0)
            : path(path), block(block), size(size), time(time)
        {}
    };

    static std::string detectType_impl(const std::string &location);

private:
    virtual IStream::pointer input_impl(File type) const UTILITY_OVERRIDE;

    virtual IStream::pointer
    input_impl(const TileId tileId, TileFile type) const UTILITY_OVERRIDE;

    virtual FileStat stat_impl(File type) const UTILITY_OVERRIDE;

    virtual FileStat stat_impl(const TileId tileId, TileFile type)
        const UTILITY_OVERRIDE;

    virtual DriverProperties properties_impl() const UTILITY_OVERRIDE {
        return { Factory::staticType(), {} };
    }

    const fs::path tarPath_;
    mutable utility::tar::Reader reader_;

    typedef std::map<TileId, Record> FileMap;
    FileMap meshMap_;
    FileMap atlasMap_;
    FileMap metaMap_;
    Record indexFile_;
    Record configFile_;
};

} } // namespace vadstena::tilestorage

#endif // vadstena_libs_tilestorage_driver_tar_hpp_included_
