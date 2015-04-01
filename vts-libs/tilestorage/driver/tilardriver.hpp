#ifndef vadstena_libs_tilestorage_driver_tilardriver_hpp_included_
#define vadstena_libs_tilestorage_driver_tilardriver_hpp_included_

#include <set>
#include <map>

#include "./tilardriver/options.hpp"
#include "./tilardriver/cache.hpp"
#include "./factory.hpp"
#include "../streams.hpp"

namespace vadstena { namespace tilestorage {

namespace fs = boost::filesystem;

class TilarDriver : public Driver {
public:
    /** Creates new storage. Existing storage is overwritten only if mode ==
     *  CreateMode::overwrite.
     */
    TilarDriver(const fs::path &root, CreateMode mode
                , const CreateProperties &properties);

    /** Opens storage.
     */
    TilarDriver(const fs::path &root, OpenMode mode
                , const DetectionContext &context);

    virtual ~TilarDriver();

    static std::string detectType_impl(DetectionContext &context
                                       , const std::string &location);

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

    virtual FileStat stat_impl(File type) const UTILITY_OVERRIDE;

    virtual FileStat stat_impl(const TileId tileId, TileFile type)
        const UTILITY_OVERRIDE;

    virtual Resources resources_impl() const UTILITY_OVERRIDE;

    virtual void begin_impl() UTILITY_OVERRIDE;

    virtual void commit_impl() UTILITY_OVERRIDE;

    virtual void rollback_impl() UTILITY_OVERRIDE;

    virtual void flush_impl() UTILITY_OVERRIDE;

    virtual void drop_impl() UTILITY_OVERRIDE;

    virtual bool externallyChanged_impl() const UTILITY_OVERRIDE;

    virtual DriverProperties properties_impl() const UTILITY_OVERRIDE;

    void writeExtraFiles();

    /** Backing root.
     */
    const fs::path root_;

    /** temporary files backing root.
     */
    const fs::path tmp_;

    /** Path to mapConfig
     */
    fs::path mapConfigPath_;

    tilardriver::Options options_;

    mutable tilardriver::Cache cache_;

    struct Tx {
        typedef std::set<fs::path> Files;
        Files files;
    };

    boost::optional<Tx> tx_;

    /** Information about mapConfig when tileset was open in read-only mode.
     */
    FileStat openStat_;
};

} } // namespace vadstena::tilestorage

#endif // vadstena_libs_tilestorage_driver_tilardriver_hpp_included_
