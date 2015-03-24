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
        virtual Driver::pointer                                         \
        create(const std::string location, CreateMode mode              \
               , const Driver::CreateProperties &properties)            \
            const UTILITY_OVERRIDE                                      \
        {                                                               \
            return std::make_shared<DRIVER_CLASS>                       \
                (location, mode, properties);                           \
        }                                                               \
                                                                        \
        virtual Driver::pointer open(const std::string location         \
                                     , OpenMode mode)                   \
            const UTILITY_OVERRIDE                                      \
        {                                                               \
            return std::make_shared<DRIVER_CLASS>(location, mode);      \
        }                                                               \
                                                                        \
        virtual std::string help() const UTILITY_OVERRIDE               \
        {                                                               \
            return DRIVER_CLASS::help;                                  \
        }                                                               \
                                                                        \
        virtual std::string detectType(const std::string &location      \
                                       , std::set<std::string> &context) \
            const UTILITY_OVERRIDE                                      \
        {                                                               \
            return DRIVER_CLASS::detectType_impl(location, context);    \
        }                                                               \
                                                                        \
        static const char* staticType() UTILITY_OVERRIDE                \
        {                                                               \
            return DRIVER_TYPE;                                         \
        }                                                               \
    }

} } // namespace vadstena::tilestorage

#endif // vadstena_libs_tilestorage_driver_factory_hpp_included_
