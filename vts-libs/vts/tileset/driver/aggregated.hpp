#ifndef vadstena_libs_vts_tileset_driver_aggregated_hpp_included_
#define vadstena_libs_vts_tileset_driver_aggregated_hpp_included_

#include <set>
#include <map>
#include <memory>

#include "../driver.hpp"
#include "../../storage.hpp"

namespace vadstena { namespace vts { namespace driver {

/** Helper class.
 */
struct AggregatedDriverBase {
    AggregatedDriverBase() {}
    AggregatedDriverBase(const CloneOptions &cloneOptions);
};

class AggregatedDriver : private AggregatedDriverBase, public Driver {
public:
    typedef std::shared_ptr<AggregatedDriver> pointer;

    /** Creates new storage. Existing storage is overwritten only if mode ==
     *  CreateMode::overwrite.
     */
    AggregatedDriver(const boost::filesystem::path &root
                     , const AggregatedOptions &options
                     , const CloneOptions &cloneOptions);

    /** Creates in-memory storage.
     */
    AggregatedDriver(const AggregatedOptions &options
                     , const CloneOptions &cloneOptions);

    /** Opens storage.
     */
    AggregatedDriver(const boost::filesystem::path &root
                     , const AggregatedOptions &options);

    /** Cloner
     */
    AggregatedDriver(const boost::filesystem::path &root
                     , const AggregatedOptions &options
                     , const CloneOptions &cloneOptions
                     , const AggregatedDriver &src);

    virtual ~AggregatedDriver();

    typedef std::uint16_t TilesetReference;

    typedef std::vector<TilesetReference> TilesetReferences;

    typedef std::vector<TilesetReferences> TilesetReferencesList;

    struct DriverEntry {
        Driver::pointer driver;
        TilesetReferences tilesets;

        typedef std::vector<DriverEntry> list;

        DriverEntry(const Driver::pointer &driver
                    , const TilesetReferences &tilesets)
            : driver(driver), tilesets(tilesets)
        {}
    };

private:
    virtual OStream::pointer output_impl(const File type);

    virtual IStream::pointer input_impl(File type) const {
        return input_impl(type, true);
    }

    virtual IStream::pointer input_impl(File type, const NullWhenNotFound_t&)
        const
    {
        return input_impl(type, false);
    }

    virtual OStream::pointer
    output_impl(const TileId &tileId, TileFile type);

    virtual IStream::pointer
    input_impl(const TileId &tileId, TileFile type) const {
        return input_impl(tileId, type, true);
    }

    virtual IStream::pointer
    input_impl(const TileId &tileId, TileFile type, const NullWhenNotFound_t&)
        const
    {
        return input_impl(tileId, type, false);
    }

    IStream::pointer input_impl(File type, bool noSuchFile) const;

    IStream::pointer input_impl(const TileId &tileId, TileFile type
                                , bool noSuchFile) const;

    virtual void drop_impl();

    virtual void flush_impl();

    virtual FileStat stat_impl(File type) const;

    virtual FileStat stat_impl(const TileId &tileId, TileFile type) const;

    virtual Resources resources_impl() const;

    Driver::pointer clone_impl(const boost::filesystem::path &root
                               , const CloneOptions &cloneOptions) const;

    virtual std::string info_impl() const;

    IStream::pointer input_mem(File type) const;

    virtual tileset::Index* getTileIndex_impl() { return &tsi_; }

    virtual const tileset::Index* getTileIndex_impl() const { return &tsi_; }

    inline const AggregatedOptions& options() const {
        return Driver::options<const AggregatedOptions&>();
    }

    TileSet::Properties build(AggregatedOptions options
                              , const CloneOptions &cloneOptions);

    Storage storage_;

    registry::ReferenceFrame referenceFrame_;

    DriverEntry::list drivers_;

    tileset::Index tsi_;

    boost::optional<TileSet::Properties> memProperties_;
};

} } } // namespace vadstena::vts::driver

#endif // vadstena_libs_vts_tileset_driver_aggregated_hpp_included_
