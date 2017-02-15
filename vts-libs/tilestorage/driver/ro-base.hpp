#ifndef vtslibs_tilestorage_driver_ro_base_hpp_included_
#define vtslibs_tilestorage_driver_ro_base_hpp_included_

#include "../driver.hpp"
#include "../../storage/error.hpp"

#include "dbglog/dbglog.hpp"

#include "utility/gccversion.hpp"

namespace vtslibs { namespace tilestorage {

namespace fs = boost::filesystem;

class ReadOnlyDriver : public Driver {
public:
    ReadOnlyDriver(CreateMode, const CreateProperties&)
        : Driver(true)
    {
        LOGTHROW(err2, storage::ReadOnlyError)
            << "This driver supports read access only.";
    }

    /** Opens storage.
     */
    ReadOnlyDriver(bool readOnly)
        : Driver(readOnly)
    {
        if (!readOnly) {
            LOGTHROW(err2, storage::ReadOnlyError)
                << "This driver supports read access only.";
        }
    }

private:
    virtual OStream::pointer output_impl(File) UTILITY_OVERRIDE {
        LOGTHROW(err2, storage::ReadOnlyError)
            << "This driver supports read access only.";
        return {};
    }

    virtual OStream::pointer
    output_impl(const TileId, TileFile) UTILITY_OVERRIDE {
        LOGTHROW(err2, storage::ReadOnlyError)
            << "This driver supports read access only.";
        return {};
    }

    virtual void remove_impl(const TileId, TileFile)
        UTILITY_OVERRIDE
    {
        LOGTHROW(err2, storage::ReadOnlyError)
            << "This driver supports read access only.";
    }

    virtual void begin_impl() UTILITY_OVERRIDE {
        LOGTHROW(err2, storage::ReadOnlyError)
            << "This driver supports read access only.";
    }

    virtual void commit_impl() UTILITY_OVERRIDE {
        LOGTHROW(err2, storage::ReadOnlyError)
            << "This driver supports read access only.";
    }

    virtual void rollback_impl() UTILITY_OVERRIDE {
        LOGTHROW(err2, storage::ReadOnlyError)
            << "This driver supports read access only.";
    }

    virtual void drop_impl() UTILITY_OVERRIDE {
        LOGTHROW(err2, storage::ReadOnlyError)
            << "This driver supports read access only.";
    }
};

} } // namespace vtslibs::tilestorage

#endif // vtslibs_tilestorage_driver_ro_base_hpp_included_
