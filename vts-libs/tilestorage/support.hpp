/**
 * \file tilestorage/support.hpp
 * \author Vaclav Blazek <vaclav.blazek@citationtech.net>
 *
 * Support files
 */

#ifndef vadstena_libs_tilestorage_support_hpp_included_
#define vadstena_libs_tilestorage_support_hpp_included_

#include <cstddef>
#include <ctime>
#include <string>
#include <map>

namespace vadstena { namespace tilestorage {

struct SupportFile {
    const unsigned char *data;
    std::size_t size;
    std::time_t lastModified;
    const char *contentType;

    typedef std::map<std::string, SupportFile> Files;
    static const Files files;
};

} } // namespace vadstena::tilestorage

#endif // vadstena_libs_tilestorage_support_hpp_included_
