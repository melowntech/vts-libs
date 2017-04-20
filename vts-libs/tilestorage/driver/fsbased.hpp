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
#ifndef vtslibs_tilestorage_driver_fsbased_hpp_included_
#define vtslibs_tilestorage_driver_fsbased_hpp_included_

#include <set>
#include <map>

#include <boost/optional.hpp>
#include <boost/filesystem/path.hpp>

#include "../driver.hpp"
#include "./factory.hpp"
#include "../../storage/fstreams.hpp"

namespace vtslibs { namespace tilestorage {

namespace fs = boost::filesystem;

class FsBasedDriver : public Driver {
public:
    /** Creates new storage. Existing storage is overwritten only if mode ==
     *  CreateMode::overwrite.
     */
    FsBasedDriver(const fs::path &root, CreateMode mode
                  , const CreateProperties &properties);

    /** Opens storage.
     */
    FsBasedDriver(const fs::path &root, OpenMode mode
                  , const DetectionContext &context);

    virtual ~FsBasedDriver();

    static std::string detectType_impl(DetectionContext &context
                                       , const std::string &location);

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

    virtual FileStat stat_impl(File type) const UTILITY_OVERRIDE;

    virtual FileStat stat_impl(const TileId tileId, TileFile type)
        const UTILITY_OVERRIDE;

    virtual void begin_impl() UTILITY_OVERRIDE;

    virtual void commit_impl() UTILITY_OVERRIDE;

    virtual void rollback_impl() UTILITY_OVERRIDE;

    virtual void drop_impl() UTILITY_OVERRIDE;

    virtual bool externallyChanged_impl() const UTILITY_OVERRIDE;

    virtual void postOpenCheck(const DetectionContext &context)
        UTILITY_OVERRIDE;

    fs::path fileDir(File type, const fs::path &name) const;

    fs::path fileDir(const TileId &tileId, TileFile type
                     , const fs::path &name) const;

    fs::path readPath(const fs::path &dir, const fs::path &name) const;

    std::pair<fs::path, storage::OnClose>
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

    /** Information about mapConfig when tileset was open in read-only mode.
     */
    FileStat openStat_;
    fs::path mapConfigPath_;
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

} } // namespace vtslibs::tilestorage

#endif // vtslibs_tilestorage_driver_fsbased_hpp_included_
