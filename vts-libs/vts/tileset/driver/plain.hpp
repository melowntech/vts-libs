#ifndef vtslibs_vts_tileset_driver_plain_hpp_included_
#define vtslibs_vts_tileset_driver_plain_hpp_included_

#include <set>
#include <map>
#include <memory>

#include "../driver.hpp"
#include "./cache.hpp"

namespace vtslibs { namespace vts { namespace driver {

class PlainDriver : public Driver {
public:
    typedef std::shared_ptr<PlainDriver> pointer;

    /** Creates new storage. Existing storage is overwritten only if mode ==
     *  CreateMode::overwrite.
     */
    PlainDriver(const boost::filesystem::path &root
                , const PlainOptions &options
                , const CloneOptions &cloneOptions);

    /** Opens storage.
     */
    PlainDriver(const boost::filesystem::path &root
                , const OpenOptions &openOptions
                , const PlainOptions &options);

    virtual ~PlainDriver();

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

    virtual Driver::pointer
    clone_impl(const boost::filesystem::path &root
               , const CloneOptions &cloneOptions) const;

    virtual std::string info_impl() const;

    mutable driver::Cache cache_;
};

} } } // namespace vtslibs::vts::driver

#endif // vtslibs_vts_tileset_driver_plain_hpp_included_
