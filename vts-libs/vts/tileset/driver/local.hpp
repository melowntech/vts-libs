#ifndef vadstena_libs_vts_tileset_driver_local_hpp_included_
#define vadstena_libs_vts_tileset_driver_local_hpp_included_

#include <set>
#include <map>
#include <memory>

#include "../driver.hpp"

namespace vadstena { namespace vts { namespace driver {

/** Helper class.
 */
struct LocalDriverBase {
    LocalDriverBase() {}
    LocalDriverBase(const CloneOptions &cloneOptions);
};

class LocalDriver : private LocalDriverBase, public Driver {
public:
    typedef std::shared_ptr<LocalDriver> pointer;

    /** Creates new storage. Existing storage is overwritten only if mode ==
     *  CreateMode::overwrite.
     */
    LocalDriver(const boost::filesystem::path &root
                     , const LocalOptions &options
                     , const CloneOptions &cloneOptions);

    /** Opens storage.
     */
    LocalDriver(const boost::filesystem::path &root
                , const OpenOptions &openOptions
                , const LocalOptions &options);

    virtual ~LocalDriver();

private:
    virtual OStream::pointer output_impl(const File type);

    virtual IStream::pointer input_impl(File type) const;

    virtual IStream::pointer input_impl(File type, const NullWhenNotFound_t&)
        const;

    virtual OStream::pointer
    output_impl(const TileId &tileId, TileFile type);

    virtual IStream::pointer
    input_impl(const TileId &tileId, TileFile type) const;

    virtual IStream::pointer
    input_impl(const TileId &tileId, TileFile type, const NullWhenNotFound_t&)
        const;

    virtual void drop_impl();

    virtual void flush_impl();

    virtual FileStat stat_impl(File type) const;

    virtual FileStat stat_impl(const TileId &tileId, TileFile type) const;

    virtual Resources resources_impl() const;

    Driver::pointer clone_impl(const boost::filesystem::path &root
                               , const CloneOptions &cloneOptions) const;

    virtual std::string info_impl() const;

    inline const LocalOptions& options() const {
        return Driver::options<const LocalOptions&>();
    }

    Driver::pointer driver_;
};

} } } // namespace vadstena::vts::driver

#endif // vadstena_libs_vts_tileset_driver_local_hpp_included_
