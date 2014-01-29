#include "dbglog/dbglog.hpp"

#include "../tilestorage.hpp"
#include "./fs.hpp"

namespace vadstena { namespace tilestorage {

namespace {
void storageDeleter(Storage *storage)
{
    storage->flush();
    delete storage;
}

struct Locator {
    std::string type;
    std::string location;
};

/** TODO: implement
 */
Locator parseUri(const std::string &uri)
{
    return { "file", uri };
}

} // namespace

struct Storage::Factory
{
    static Storage::pointer create(const std::string &uri
                                   , const CreateProperties &properties
                                   , CreateMode mode)
    {
        auto locator(parseUri(uri));

        Storage *storage(nullptr);

        if (locator.type == "file") {
            storage = new FileSystemStorage
                (locator.location, properties, mode);
        } else {
            LOGTHROW(err2, NoSuchStorage)
                << "Invalid storage type <" << locator.type << ">.";
        }

        return { storage, &storageDeleter };
    }

    static Storage::pointer open(const std::string &uri, OpenMode mode)
    {
        auto locator(parseUri(uri));

        Storage *storage(nullptr);

        if (locator.type == "file") {
            storage = new FileSystemStorage(locator.location, mode);
        } else {
            LOGTHROW(err2, NoSuchStorage)
                << "Invalid storage type <" << locator.type << ">.";
        }

        return { storage, &storageDeleter };
    }
};

Storage::pointer create(const std::string &uri
                        , const CreateProperties &properties
                        , CreateMode mode)
{
    return Storage::Factory::create(uri, properties, mode);
}

Storage::pointer open(const std::string &uri, OpenMode mode)
{
    return Storage::Factory::open(uri, mode);
}

Storage::~Storage()
{
}

} } // namespace vadstena::tilestorage
