#ifndef vadstena_libs_vts_tileset_driver_hpp_included_
#define vadstena_libs_vts_tileset_driver_hpp_included_

#include <set>
#include <map>
#include <memory>

#include <boost/noncopyable.hpp>

#include "utility/runnable.hpp"

#include "../../storage/streams.hpp"
#include "../../storage/resources.hpp"

#include "../options.hpp"
#include "./driver/options.hpp"

namespace vadstena { namespace vts {

using storage::OStream;
using storage::IStream;
using storage::File;
using storage::TileFile;
using storage::FileStat;
using storage::Resources;

class Driver : boost::noncopyable {
public:
    typedef std::shared_ptr<Driver> pointer;

    virtual ~Driver();

    /** Creates driver for new dataset.
     */
    static pointer create(const boost::filesystem::path &root
                          , const boost::any &options
                          , const CloneOptions &cloneOptions);

    /** Opens driver for existing dataset.
     */
    static pointer open(const boost::filesystem::path &root);

    /** Creates driver for new dataset as exact copy of the original one.
     */
    pointer clone(const boost::filesystem::path &root
                  , const CloneOptions &cloneOptions) const;

    OStream::pointer output(File type);

    IStream::pointer input(File type) const;

    OStream::pointer output(const TileId &tileId, TileFile type);

    IStream::pointer input(const TileId &tileId, TileFile type) const;

    FileStat stat(File type) const;

    FileStat stat(const TileId &tileId, TileFile type) const;

    Resources resources() const;

    void flush();

    bool externallyChanged() const;

    const boost::any& options() const { return options_; }

    template <typename T>
    const T& options() const { return boost::any_cast<const T&>(options_); }

    template <typename T>
    static const T& options(const boost::any &options) {
        return boost::any_cast<const T&>(options);
    }

    void wannaWrite(const std::string &what) const;

    /** Drop storage.
     */
    void drop();

    /** Sets runnable that is observed during access operations.
     *  If stopped runnable is encountered operation throws Interrupted.
     *  Pass nullptr to stop watching runnable.
     */
    void watch(utility::Runnable *runnable);

    /** Gets old revision (if any)
     */
    boost::optional<unsigned int> oldRevision() {
        return oldRevision_;
    }

    const boost::filesystem::path& root() const { return root_; }

    /** Returns time of last modification. Recorded at read-only open.
     */
    std::time_t lastModified() const { return lastModified_; }

    bool readOnly() const;

    /** Information.
     */
    std::string info() const;

protected:
    /** Creates new storage. Existing storage is overwritten only if mode ==
     *  CreateMode::overwrite.
     */
    Driver(const boost::filesystem::path &root, const boost::any &options
           , CreateMode mode);

    /** Opens storage.
     */
    Driver(const boost::filesystem::path &root
           , const boost::any &options);

    void readOnly(bool value);

    const FileStat& configStat() const { return configStat_; }

private:
    virtual OStream::pointer output_impl(const File type) = 0;

    virtual IStream::pointer input_impl(File type) const = 0;

    virtual OStream::pointer
    output_impl(const TileId &tileId, TileFile type) = 0;

    virtual IStream::pointer
    input_impl(const TileId &tileId, TileFile type) const = 0;

    virtual void drop_impl() = 0;

    virtual void flush_impl() = 0;

    virtual FileStat stat_impl(File type) const = 0;

    virtual FileStat stat_impl(const TileId &tileId, TileFile type)
        const = 0;

    virtual Resources resources_impl() const = 0;

    virtual pointer clone_impl(const boost::filesystem::path &root
                               , const CloneOptions &cloneOptions) const = 0;

    virtual std::string info_impl() const = 0;

    void checkRunning() const;

    void notRunning() const;

    /** Backing root.
     */
    const boost::filesystem::path root_;

    bool readOnly_;

    /** Path to config
     */
    boost::filesystem::path configPath_;

    /** Path to extra-config
     */
    boost::filesystem::path extraConfigPath_;

    boost::any options_;

    /** Information about root when tileset was open in read-only mode.
     */
    FileStat rootStat_;

    /** Information about config when tileset was open in read-only mode.
     */
    FileStat configStat_;

    /** Information about extra-config when tileset was open in read-only mode.
     */
    FileStat extraConfigStat_;

    /** Runnable associated with the driver.
     */
    utility::Runnable *runnable_;

    /** Revision read from old config (if any).
     */
    boost::optional<unsigned int> oldRevision_;

    /** Time of last modification (recorded at read-only open)
     */
    std::time_t lastModified_;
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

inline OStream::pointer Driver::output(File type)
{
    checkRunning();
    return output_impl(type);
}

inline IStream::pointer Driver::input(File type) const
{
    checkRunning();
    return input_impl(type);
}

inline OStream::pointer Driver::output(const TileId &tileId, TileFile type)
{
    checkRunning();
    return output_impl(tileId, type);
}

inline IStream::pointer Driver::input(const TileId &tileId, TileFile type)
    const
{
    checkRunning();
    return input_impl(tileId, type);
}

inline FileStat Driver::stat(File type) const
{
    checkRunning();
    return stat_impl(type);
}

inline FileStat Driver::stat(const TileId &tileId, TileFile type) const
{
    checkRunning();
    return stat_impl(tileId, type);
}

inline storage::Resources Driver::resources() const
{
    return resources_impl();
}

inline void Driver::drop()
{
    checkRunning();
    return drop_impl();
}

inline void Driver::flush()
{
    return flush_impl();
}

inline Driver::pointer Driver::clone(const boost::filesystem::path &root
                                     , const CloneOptions &cloneOptions)
    const
{
    return clone_impl(root, cloneOptions);
}

inline std::string Driver::info() const
{
    return info_impl();
}

} } // namespace vadstena::vts

#endif // vadstena_libs_vts_tileset_driver_hpp_included_
