#include "utility/uri.hpp"

#include "../tilestorage.hpp"
#include "./fs.hpp"

namespace vadstena { namespace tilestorage {

Storage::pointer create(const std::string &uri)
{
    auto u(utility::parseUri(uri));
    if (u.schema == "file") {
        return FileSystemStorage::create(u.path);
    }

    return {};
}

Storage::pointer open(const std::string &uri, OpenMode mode)
{
    auto u(utility::parseUri(uri));
    if (u.schema == "file") {
        return FileSystemStorage::open(u.path, mode);
    }

    return {};
}

} } // namespace vadstena::tilestorage
