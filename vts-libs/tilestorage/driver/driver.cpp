/**
 * Copyright (c) 2017 Melown Technologies SE
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * *  Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#include <mutex>

#include "../io.hpp"
#include "../driver.hpp"
#include "../../storage/error.hpp"
#include "./flat.hpp"
#include "./hash-crc.hpp"
#include "./tar.hpp"
#include "./tilardriver.hpp"
#include "../config.hpp"

namespace vtslibs { namespace tilestorage {

namespace fs = boost::filesystem;

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
        LOGTHROW(err2, storage::ReadOnlyError)
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
        LOGTHROW(err2, storage::NoSuchTileSet)
            << "Invalid tile set type <" << locator.type << ">.";
    }
    return fregistry->second->create(locator.location, mode, properties);
}

Driver::pointer Driver::open(Locator locator, OpenMode mode)
{
    registerDefaultDrivers();

    DetectionContext context;
    if (locator.type.empty()) {
        // no type specified -> try to locate config file and pull in options
        locator.type = detectType(context, locator.location);
    }
    if (locator.type.empty()) {
        // cannot detect -> try default driver
        locator.type = DefaultDriver;
    }

    auto fregistry(driverRegistry.find(locator.type));
    if (fregistry == driverRegistry.end()) {
        LOGTHROW(err2, storage::NoSuchTileSet)
            << "Invalid tile set type <" << locator.type << ">.";
    }

    auto driver(fregistry->second->open(locator.location, mode, context));
    driver->postOpenCheck(context);
    return driver;
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

std::string Driver::detectType(DetectionContext &context
                               , const std::string &location)
{
    for (const auto &pair : driverRegistry) {
        const auto type = pair.second->detectType(context, location);
        if (!type.empty()) { return type; }
    }

    return {};
}

void Driver::notRunning() const
{
    LOGTHROW(warn2, storage::Interrupted)
        << "Operation has been interrupted.";
}

bool DetectionContext::seen(const std::string &token) const
{
    return map_.find(token) != map_.end();
}

bool DetectionContext::mark(const std::string &token)
{
    return map_.insert(Map::value_type(token, boost::any())).second;
}

bool DetectionContext::mark(const std::string &token, const boost::any value)
{
    return map_.insert(Map::value_type(token, value)).second;
}

boost::any DetectionContext::getValue(const std::string &token) const
{
    auto fmap(map_.find(token));
    if (fmap != map_.end()) { return fmap->second; }
    return {};
}

std::string Driver::detectTypeFromMapConfig(DetectionContext &context
                                            , const fs::path &path)
{
    try {
        // try load config
        auto tmp(absolute(path));
        if (context.seen(tmp.string())) { return {}; }
        auto config(tilestorage::loadConfig(tmp));
        context.mark(tmp.string(), config);
        return config.driver.type;
    } catch (const std::exception &e) {}
    return {};

}

} } // namespace vtslibs::tilestorage
