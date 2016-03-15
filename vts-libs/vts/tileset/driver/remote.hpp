#ifndef vadstena_libs_vts_tileset_driver_remote_hpp_included_
#define vadstena_libs_vts_tileset_driver_remote_hpp_included_

#include <set>
#include <map>
#include <memory>

#include "../driver.hpp"
#include "./httpfetcher.hpp"

namespace vadstena { namespace vts { namespace driver {

/** Helper class.
 */
struct RemoteDriverBase {
    RemoteDriverBase() {}
    RemoteDriverBase(const CloneOptions &cloneOptions);
};

class RemoteDriver : private RemoteDriverBase, public Driver {
public:
    typedef std::shared_ptr<RemoteDriver> pointer;

    /** Creates new storage. Existing storage is overwritten only if mode ==
     *  CreateMode::overwrite.
     */
    RemoteDriver(const boost::filesystem::path &root
                     , const RemoteOptions &options
                     , const CloneOptions &cloneOptions);

    /** Opens storage.
     */
    RemoteDriver(const boost::filesystem::path &root
                     , const RemoteOptions &options);

    /** Cloner
     */
    RemoteDriver(const boost::filesystem::path &root
                     , const RemoteOptions &options
                     , const CloneOptions &cloneOptions
                     , const RemoteDriver &src);

    virtual ~RemoteDriver();

private:
    virtual OStream::pointer output_impl(const File type);

    virtual IStream::pointer input_impl(File type) const;

    virtual OStream::pointer
    output_impl(const TileId &tileId, TileFile type);

    virtual IStream::pointer
    input_impl(const TileId &tileId, TileFile type) const;

    virtual void drop_impl();

    virtual void flush_impl();

    virtual FileStat stat_impl(File type) const;

    virtual FileStat stat_impl(const TileId &tileId, TileFile type) const;

    virtual Resources resources_impl() const;

    Driver::pointer clone_impl(const boost::filesystem::path &root
                               , const CloneOptions &cloneOptions) const;

    virtual std::string info_impl() const;

    inline const RemoteOptions& options() const {
        return Driver::options<const RemoteOptions&>();
    }

    HttpFetcher fetcher_;
    unsigned int revision_;
    tileset::Index tsi_;
};

} } } // namespace vadstena::vts::driver

#endif // vadstena_libs_vts_tileset_driver_remote_hpp_included_
