#include <mutex>

#include "../driver.hpp"
#include "../error.hpp"
#include "./flat.hpp"
#include "./hash-crc.hpp"
#include "./tar.hpp"
#include "./tilardriver.hpp"

namespace vadstena { namespace tilestorage {

namespace {

const char* DefaultDriver("flat");

std::map<std::string, Driver::Factory::pointer> driverRegistry;

std::once_flag driverRegistryOnceFlag;

void registerDriversOnce()
{
    // add drivers here
    Driver::registerDriver<FlatDriver>();
    Driver::registerDriver<HashCrcDriver>();
    Driver::registerDriver<TarDriver>();
    Driver::registerDriver<TilarDriver>();
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

Driver::pointer Driver::create(Locator locator, CreateMode mode
                               , const CreateProperties &properties)
{
    registerDefaultDrivers();

    if (locator.type.empty()) {
        if (!properties->driver.type.empty()) {
            locator.type = properties->driver.type;
        } else {
            locator.type = DefaultDriver;
        }
    }

    auto fregistry(driverRegistry.find(locator.type));
    if (fregistry == driverRegistry.end()) {
        LOGTHROW(err2, NoSuchTileSet)
            << "Invalid tile set type <" << locator.type << ">.";
    }
    return fregistry->second->create(locator.location, mode, properties);
}

Driver::pointer Driver::open(Locator locator, OpenMode mode)
{
    registerDefaultDrivers();

    if (locator.type.empty()) {
        // no type specified -> try to locate config file and pull in options
        locator.type = detectType(locator.location);
    }
    if (locator.type.empty()) {
        // cannot detect -> try default driver
        locator.type = DefaultDriver;
    }

    auto fregistry(driverRegistry.find(locator.type));
    if (fregistry == driverRegistry.end()) {
        LOGTHROW(err2, NoSuchTileSet)
            << "Invalid tile set type <" << locator.type << ">.";
    }
    return fregistry->second->open(locator.location, mode);
}

std::map<std::string, std::string> Driver::listSupportedDrivers()
{
    registerDefaultDrivers();

    std::map<std::string, std::string> list;
    for (const auto &pair : driverRegistry) {
        list.insert(std::make_pair(pair.first, pair.second->help()));
    }
    return list;
}

std::string Driver::detectType(const std::string &location)
{
    (void) location;

    for (const auto &pair : driverRegistry) {
        const auto type = pair.second->detectType(location);
        if (!type.empty()) { return type; }
    }

    return {};
}

void Driver::notRunning() const
{
    LOGTHROW(warn2, Interrupted)
        << "Transaction has been interrupted.";
}

} } // namespace vadstena::tilestorage
