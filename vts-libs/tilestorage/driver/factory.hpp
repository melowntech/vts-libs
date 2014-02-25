#ifndef vadstena_libs_tilestorage_driver_factory_hpp_included_
#define vadstena_libs_tilestorage_driver_factory_hpp_included_

#include "utility/gccversion.hpp"

#include "../driver.hpp"

namespace vadstena { namespace tilestorage {

#define VADSTENA_TILESTORAGE_DRIVER_FACTORY(DRIVER_TYPE, DRIVER_CLASS)  \
    class Factory : public Driver::Factory {                            \
    public:                                                             \
        Factory() : Driver::Factory(DRIVER_TYPE) {}                     \
                                                                        \
        virtual Driver::pointer create(const std::string location       \
                                       , CreateMode mode)               \
            const UTILITY_OVERRIDE                                      \
        {                                                               \
            return std::make_shared<DRIVER_CLASS>(location, mode);      \
        }                                                               \
                                                                        \
        virtual Driver::pointer open(const std::string location         \
                                     , OpenMode mode)                   \
            const UTILITY_OVERRIDE                                      \
        {                                                               \
            return std::make_shared<DRIVER_CLASS>(location, mode);      \
        }                                                               \
    }

} } // namespace vadstena::tilestorage

#endif // vadstena_libs_tilestorage_driver_factory_hpp_included_
