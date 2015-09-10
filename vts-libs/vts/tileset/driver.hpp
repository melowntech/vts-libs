#ifndef vadstena_libs_vts_tileset_driver_hpp_included_
#define vadstena_libs_vts_tileset_driver_hpp_included_

#include <set>
#include <map>
#include <memory>

#include "utility/runnable.hpp"

#include "../../storage/streams.hpp"
#include "../../storage/resources.hpp"

#include "./driver/options.hpp"
#include "./driver/cache.hpp"

namespace vadstena { namespace vts {

using storage::OStream;
using storage::IStream;
using storage::File;
using storage::TileFile;
using storage::FileStat;
using storage::Resources;

class Driver {
public:
    typedef std::shared_ptr<Driver> pointer;

    /** Creates new storage. Existing storage is overwritten only if mode ==
     *  CreateMode::overwrite.
     */
    Driver(const boost::filesystem::path &root, CreateMode mode
           , const driver::Options &options);

    /** Opens storage.
     */
    Driver(const boost::filesystem::path &root);

    ~Driver();

    OStream::pointer output(File type);

    IStream::pointer input(File type) const;

    OStream::pointer output(const TileId tileId, TileFile type);

    IStream::pointer input(const TileId tileId, TileFile type) const;

    FileStat stat(File type) const;

    FileStat stat(const TileId tileId, TileFile type) const;

    Resources resources() const;

    void flush();

    bool externallyChanged() const;

    driver::Options options() const { return options_; }

    void wannaWrite(const std::string &what) const;

    /** Drop storage.
     */
    void drop();

    /** Sets runnable that is observed during access operations.
     *  If stopped runnable is encountered operation throws Interrupted.
     *  Pass nullptr to stop watching runnable.
     */
    void watch(utility::Runnable *runnable);

    /** Gets old config file content and removes any notion about it.
     */
    boost::optional<std::string> oldConfig() {
        auto c(oldConfig_); oldConfig_= boost::none; return c;
    }

private:
    void checkRunning() const;

    void notRunning() const;

    /** Backing root.
     */
    const boost::filesystem::path root_;

    /** Path to mapConfig
     */
    boost::filesystem::path configPath_;

    driver::Options options_;

    mutable driver::Cache cache_;

    /** Information about mapConfig when tileset was open in read-only mode.
     */
    FileStat openStat_;

    /** Runnable associated with the driver.
     */
    utility::Runnable *runnable_;

    /** Content of old config file.
     */
    boost::optional<std::string> oldConfig_;
};

inline void Driver::watch(utility::Runnable *runnable)
{
    runnable_ = runnable;
}

inline void Driver::checkRunning() const
{
    if (!runnable_ || *runnable_) { return; }
    notRunning();
}

} } // namespace vadstena::vts

#endif // vadstena_libs_vts_tileset_driver_hpp_included_
