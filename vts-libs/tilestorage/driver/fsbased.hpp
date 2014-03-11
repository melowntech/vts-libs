#ifndef vadstena_libs_tilestorage_driver_fsbased_hpp_included_
#define vadstena_libs_tilestorage_driver_fsbased_hpp_included_

#include <map>

#include <boost/optional.hpp>
#include <boost/filesystem/path.hpp>

#include "../driver.hpp"
#include "./factory.hpp"

namespace vadstena { namespace tilestorage {

class FsBasedDriver : public Driver {
public:
    /** Creates new storage. Existing storage is overwritten only if mode ==
     *  CreateMode::overwrite.
     */
    FsBasedDriver(const boost::filesystem::path &root, CreateMode mode);

    /** Opens storage.
     */
    FsBasedDriver(const boost::filesystem::path &root, OpenMode mode);

    virtual ~FsBasedDriver();

private:
    /** Implement in derived class.
     */
    virtual boost::filesystem::path fileDir_impl(File type) const = 0;

    /** Implement in derived class.
     */
    virtual boost::filesystem::path fileDir_impl(const TileId &tileId
                                                 , TileFile type) const = 0;

    virtual OStream::pointer output_impl(File type) UTILITY_OVERRIDE;

    virtual IStream::pointer input_impl(File type) const UTILITY_OVERRIDE;

    virtual OStream::pointer
    output_impl(const TileId tileId, TileFile type) UTILITY_OVERRIDE;

    virtual IStream::pointer
    input_impl(const TileId tileId, TileFile type) const UTILITY_OVERRIDE;

    virtual void begin_impl() UTILITY_OVERRIDE;

    virtual void commit_impl() UTILITY_OVERRIDE;

    virtual void rollback_impl() UTILITY_OVERRIDE;

    boost::filesystem::path fileDir(File type) const;

    boost::filesystem::path fileDir(const TileId &tileId, TileFile type) const;

    boost::filesystem::path readPath(const boost::filesystem::path &dir
                                     , const boost::filesystem::path &name)
        const;

    boost::filesystem::path writePath(const boost::filesystem::path &dir
                                      , const boost::filesystem::path &name);

    /** Backing root.
     */
    const boost::filesystem::path root_;

    /** temporary files backing root.
     */
    const boost::filesystem::path tmp_;

    /** Maps filename to its directory.
     */
    typedef std::map<boost::filesystem::path, boost::filesystem::path> TxFiles;

    /** Files in pending transaction.
     */
    boost::optional<TxFiles> txFiles_;
};


// inline implementation

inline boost::filesystem::path FsBasedDriver::fileDir(File type) const
{
    return fileDir_impl(type);
}

inline boost::filesystem::path
FsBasedDriver::fileDir(const TileId &tileId, TileFile type) const
{
    return fileDir_impl(tileId, type);
}

} } // namespace vadstena::tilestorage

#endif // vadstena_libs_tilestorage_driver_fsbased_hpp_included_
