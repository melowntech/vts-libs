#include "utility/uri.hpp"

#include "../tilestorage.hpp"
#include "./fs.hpp"

namespace vadstena { namespace tilestorage {

namespace {
void storageDeleter(Storage *storage)
{
    storage->flush();
    delete storage;
}

} // namespace

struct Storage::Factory
{
    static Storage::pointer create(const std::string &uri
                                   , const Properties &properties
                                   , CreateMode mode)
    {
        Storage *storage(nullptr);

        auto u(utility::parseUri(uri));
        if (u.schema == "file") {
            storage = new FileSystemStorage(u.path, properties, mode);
        } else {
            // TODO: throw error?
            return {};
        }

        return { storage, &storageDeleter };
    }

    static Storage::pointer open(const std::string &uri, OpenMode mode)
    {
        Storage *storage(nullptr);

        auto u(utility::parseUri(uri));
        if (u.schema == "file") {
            storage = new FileSystemStorage(u.path, mode);
        } else {
            // TODO: throw error?
            return {};
        }

        return { storage, &storageDeleter };
    }
};

Storage::pointer create(const std::string &uri, const Properties &properties
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
