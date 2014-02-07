#include <mutex>

#include "../driver.hpp"
#include "../error.hpp"
#include "./flat.hpp"

namespace vadstena { namespace tilestorage {

namespace {

const char* DefaultDriver("flat");

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
        LOGTHROW(err2, ReadOnlyError)
            << "Cannot " << what << ": storage is read-only.";
    }
}

void Driver::registerDriver(const Driver::Factory::pointer &factory)
{
    driverRegistry[factory->type] = factory;
}

Driver::pointer Driver::create(Locator locator
                               , const CreateProperties &properties
                               , CreateMode mode)
{
    registerDefaultDrivers();

    if (locator.type.empty()) {
        locator.type = DefaultDriver;
    }

    auto fregistry(driverRegistry.find(locator.type));
    if (fregistry == driverRegistry.end()) {
        LOGTHROW(err2, NoSuchTileSet)
            << "Invalid tile set type <" << locator.type << ">.";
    }
    return fregistry->second->create(locator.location, properties, mode);
}

Driver::pointer Driver::open(Locator locator
                             , OpenMode mode)
{
    registerDefaultDrivers();

    if (locator.type.empty()) {
        locator.type = DefaultDriver;
    }

    auto fregistry(driverRegistry.find(locator.type));
    if (fregistry == driverRegistry.end()) {
        LOGTHROW(err2, NoSuchTileSet)
            << "Invalid tile set type <" << locator.type << ">.";
    }
    return fregistry->second->open(locator.location, mode);
}

} } // namespace vadstena::tilestorage
