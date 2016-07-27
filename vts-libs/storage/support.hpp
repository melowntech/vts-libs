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
    typedef std::map<std::string, SupportFile> Files;
    typedef std::map<std::string, std::string> Vars;

    const unsigned char *data;
    std::size_t size;
    std::time_t lastModified;
    const char *contentType;
    bool isTemplate;

    SupportFile(const unsigned char *data, std::size_t size
                , std::time_t lastModified, const char *contentType
                , bool isTemplate = false)
        : data(data), size(size), lastModified(lastModified)
        , contentType(contentType), isTemplate(isTemplate)
    {}

    std::string expand(const Vars *vars, const Vars *defaults) const;
};

} } // namespace vadstena::storage

#endif // vadstena_libs_storage_support_hpp_included_
