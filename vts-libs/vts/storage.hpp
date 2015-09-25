/**
 * \file vts/storage.hpp
 * \author Vaclav Blazek <vaclav.blazek@citationtech.net>
 *
 * Tile set storage access.
 */

#ifndef vadstena_libs_vts_storage_hpp_included_
#define vadstena_libs_vts_storage_hpp_included_

#include <memory>
#include <string>

#include <boost/optional.hpp>
#include <boost/noncopyable.hpp>
#include <boost/filesystem/path.hpp>

#include "./tileset.hpp"

namespace vadstena { namespace vts {

struct StorageProperties {
    std::string referenceFrame;
};

/** Storage interface.
 */
class Storage {
public:
    /** Opens existing storage.
     */
    Storage(const boost::filesystem::path &path, OpenMode mode);

    /** Creates new storage.
     */
    Storage(const boost::filesystem::path &path
            , const StorageProperties &properties
            , CreateMode mode);

    ~Storage();

    /** Internals. Public to ease library developers' life, not to allow users
     *  to put their dirty hands in the storage's guts!
     */
    struct Detail;

private:
    struct DetailDeleter { void operator()(Detail*); };
    std::unique_ptr<Detail, DetailDeleter> detail_;
    Detail& detail() { return *detail_; }
    const Detail& detail() const { return *detail_; }

public:
    struct Properties;
};

} } // namespace vadstena::vts

#endif // vadstena_libs_vts_storage_hpp_included_
