#ifndef vtslibs_tilestorage_driver_factory_hpp_included_
#define vtslibs_tilestorage_driver_factory_hpp_included_

#include "utility/gccversion.hpp"

#include "../driver.hpp"

namespace vtslibs { namespace tilestorage {

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
                                     , OpenMode mode                    \
                                     , const DetectionContext &context) \
            const UTILITY_OVERRIDE                                      \
        {                                                               \
            return std::make_shared<DRIVER_CLASS>                       \
                (location, mode, context);                             \
        }                                                               \
                                                                        \
        virtual std::string help() const UTILITY_OVERRIDE               \
        {                                                               \
            return DRIVER_CLASS::help;                                  \
        }                                                               \
                                                                        \
        virtual std::string detectType(DetectionContext &context        \
                                       , const std::string &location)   \
            const UTILITY_OVERRIDE                                      \
        {                                                               \
            return DRIVER_CLASS::detectType_impl(context, location); \
        }                                                               \
                                                                        \
        static const char* staticType()                                 \
        {                                                               \
            return DRIVER_TYPE;                                         \
        }                                                               \
    }

} } // namespace vtslibs::tilestorage

#endif // vtslibs_tilestorage_driver_factory_hpp_included_
