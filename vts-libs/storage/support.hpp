/**
 * \file storage/support.hpp
 * \author Vaclav Blazek <vaclav.blazek@citationtech.net>
 *
 * Support files
 */

#ifndef vadstena_libs_storage_support_hpp_included_
#define vadstena_libs_storage_support_hpp_included_

#include <cstddef>
#include <ctime>
#include <string>
#include <map>

namespace vadstena { namespace storage {

struct SupportFile {
    const unsigned char *data;
    std::size_t size;
    std::time_t lastModified;
    const char *contentType;

    typedef std::map<std::string, SupportFile> Files;
};

} } // namespace vadstena::storage

#endif // vadstena_libs_storage_support_hpp_included_
