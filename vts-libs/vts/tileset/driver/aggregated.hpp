#ifndef vadstena_libs_vts_tileset_driver_aggregated_hpp_included_
#define vadstena_libs_vts_tileset_driver_aggregated_hpp_included_

#include <set>
#include <map>
#include <memory>

#include "../driver.hpp"
#include "./cache.hpp"

namespace vadstena { namespace vts { namespace driver {

class AggregatedDriver : public Driver {
public:
    typedef std::shared_ptr<Driver> pointer;

    /** Creates new storage. Existing storage is overwritten only if mode ==
     *  CreateMode::overwrite.
     */
    AggregatedDriver(const boost::filesystem::path &root
                     , const AggregatedOptions &options
                     , CreateMode mode, const TilesetId &tilesetId);

    /** Opens storage.
     */
    AggregatedDriver(const boost::filesystem::path &root
                     , const AggregatedOptions &options);

    virtual ~AggregatedDriver();

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

    inline const AggregatedOptions& options() const {
        return Driver::options<const AggregatedOptions&>();
    }

    Storage storage_;
};

} } } // namespace vadstena::vts::driver

#endif // vadstena_libs_vts_tileset_driver_aggregated_hpp_included_
