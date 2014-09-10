#ifndef vadstena_libs_tilestorage_driver_fsbased_hpp_included_
#define vadstena_libs_tilestorage_driver_fsbased_hpp_included_

#include <set>
#include <map>

#include <boost/optional.hpp>
#include <boost/filesystem/path.hpp>

#include "../driver.hpp"
#include "./factory.hpp"

namespace vadstena { namespace tilestorage {

namespace fs = boost::filesystem;

class FsBasedDriver : public Driver {
public:
    /** Creates new storage. Existing storage is overwritten only if mode ==
     *  CreateMode::overwrite.
     */
    FsBasedDriver(const fs::path &root, CreateMode mode);

    /** Opens storage.
     */
    FsBasedDriver(const fs::path &root, OpenMode mode);

    virtual ~FsBasedDriver();

    /** On-close callback
     */
    typedef std::function<void(bool)> OnClose;

private:
    /** Implement in derived class.
     */
    virtual fs::path fileDir_impl(File type, const fs::path &name) const = 0;

    /** Implement in derived class.
     */
    virtual fs::path fileDir_impl(const TileId &tileId, TileFile type
                                  , const fs::path &name) const = 0;

    virtual OStream::pointer output_impl(File type) UTILITY_OVERRIDE;

    virtual IStream::pointer input_impl(File type) const UTILITY_OVERRIDE;

    virtual OStream::pointer
    output_impl(const TileId tileId, TileFile type) UTILITY_OVERRIDE;

    virtual IStream::pointer
    input_impl(const TileId tileId, TileFile type) const UTILITY_OVERRIDE;

    virtual void remove_impl(const TileId tileId, TileFile type)
        UTILITY_OVERRIDE;

    virtual void begin_impl() UTILITY_OVERRIDE;

    virtual void commit_impl() UTILITY_OVERRIDE;

    virtual void rollback_impl() UTILITY_OVERRIDE;

    virtual void drop_impl() UTILITY_OVERRIDE;

    virtual void update_impl() UTILITY_OVERRIDE;

    fs::path fileDir(File type, const fs::path &name) const;

    fs::path fileDir(const TileId &tileId, TileFile type
                     , const fs::path &name) const;

    fs::path readPath(const fs::path &dir, const fs::path &name) const;

    std::pair<fs::path, OnClose>
    writePath(const fs::path &dir, const fs::path &name);

    fs::path removePath(const fs::path &dir, const fs::path &name);

    void writeExtraFiles();

    /** Backing root.
     */
    const fs::path root_;

    /** temporary files backing root.
     */
    const fs::path tmp_;

    class DirCache {
    public:
        DirCache(const fs::path &root) : root_(root) {}

        fs::path create(const fs::path &dir);

        fs::path path(const fs::path &dir);

    private:
        typedef std::set<fs::path> DirSet;

        const fs::path root_;
        DirSet dirs_;
    };

    DirCache dirCache_;

    struct Tx {
        Tx(const fs::path &root) : dirCache(root) {}

        struct Record {
            fs::path dir;
            bool removed;

            Record(const fs::path &dir, bool removed = false)
                : dir(dir), removed(removed)
            {}

            operator const fs::path&() const { return dir; }
        };

        typedef std::map<fs::path, Record> Files;

        Files files;
        DirCache dirCache;
    };

    /** Files in pending transaction.
     */
    boost::optional<Tx> tx_;
};


// inline implementation

inline fs::path FsBasedDriver::fileDir(File type, const fs::path &name) const
{
    return fileDir_impl(type, name);
}

inline fs::path FsBasedDriver::fileDir(const TileId &tileId, TileFile type
                                       , const fs::path &name)
    const
{
    return fileDir_impl(tileId, type, name);
}

} } // namespace vadstena::tilestorage

#endif // vadstena_libs_tilestorage_driver_fsbased_hpp_included_
