#ifndef vadstena_libs_vts_driver_ro_base_hpp_included_
#define vadstena_libs_vts_driver_ro_base_hpp_included_

#include "../driver.hpp"
#include "../error.hpp"

#include "dbglog/dbglog.hpp"

#include "utility/gccversion.hpp"

namespace vadstena { namespace vts {

namespace fs = boost::filesystem;

class ReadOnlyDriver : public Driver {
public:
    ReadOnlyDriver(CreateMode, const CreateProperties&)
        : Driver(true)
    {
        LOGTHROW(err2, ReadOnlyError)
            << "This driver supports read access only.";
    }

    /** Opens storage.
     */
    ReadOnlyDriver(bool readOnly)
        : Driver(readOnly)
    {
        if (!readOnly) {
            LOGTHROW(err2, ReadOnlyError)
                << "This driver supports read access only.";
        }
    }

private:
    virtual OStream::pointer output_impl(File) UTILITY_OVERRIDE {
        LOGTHROW(err2, ReadOnlyError)
            << "This driver supports read access only.";
        return {};
    }

    virtual OStream::pointer
    output_impl(const TileId, TileFile) UTILITY_OVERRIDE {
        LOGTHROW(err2, ReadOnlyError)
            << "This driver supports read access only.";
        return {};
    }

    virtual void remove_impl(const TileId, TileFile)
        UTILITY_OVERRIDE
    {
        LOGTHROW(err2, ReadOnlyError)
            << "This driver supports read access only.";
    }

    virtual void begin_impl() UTILITY_OVERRIDE {
        LOGTHROW(err2, ReadOnlyError)
            << "This driver supports read access only.";
    }

    virtual void commit_impl() UTILITY_OVERRIDE {
        LOGTHROW(err2, ReadOnlyError)
            << "This driver supports read access only.";
    }

    virtual void rollback_impl() UTILITY_OVERRIDE {
        LOGTHROW(err2, ReadOnlyError)
            << "This driver supports read access only.";
    }

    virtual void drop_impl() UTILITY_OVERRIDE {
        LOGTHROW(err2, ReadOnlyError)
            << "This driver supports read access only.";
    }
};

} } // namespace vadstena::vts

#endif // vadstena_libs_vts_driver_ro_base_hpp_included_
