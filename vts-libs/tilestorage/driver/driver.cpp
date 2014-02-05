#include <mutex>

#include "../driver.hpp"
#include "./flat.hpp"

namespace vadstena { namespace tilestorage {

namespace {

std::map<std::string, Driver::Factory::pointer> driverRegistry;

std::once_flag driverRegistryOnceFlag;

void registerDriversOnce()
{
    // add drivers here
    Driver::registerDriver<FlatDriver>();
}

void registerDefaultDrivers()
{
    std::call_once(driverRegistryOnceFlag, registerDriversOnce);
}

} // namespace

void Driver::wannaWrite(const std::string &what) const
{
    if (readOnly_) {
        LOGTHROW(err2, Error)
            << "Cannot " << what << ": storage is read-only.";
    }
}

void Driver::registerDriver(const Driver::Factory::pointer &factory)
{
    driverRegistry[factory->type] = factory;
}

Driver::pointer Driver::create(const std::string type
                               , const std::string location
                               , const CreateProperties &properties
                               , CreateMode mode)
{
    registerDefaultDrivers();

    auto fregistry(driverRegistry.find(type));
    if (fregistry == driverRegistry.end()) {
        LOGTHROW(err2, NoSuchTileSet)
            << "Invalid tile set type <" << type << ">.";
    }
    return fregistry->second->create(location, properties, mode);
}

Driver::pointer Driver::open(const std::string type
                             , const std::string location
                             , OpenMode mode)
{
    registerDefaultDrivers();

    auto fregistry(driverRegistry.find(type));
    if (fregistry == driverRegistry.end()) {
        LOGTHROW(err2, NoSuchTileSet)
            << "Invalid tile set type <" << type << ">.";
    }
    return fregistry->second->open(location, mode);
}

} } // namespace vadstena::tilestorage
