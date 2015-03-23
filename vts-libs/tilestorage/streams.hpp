#ifndef vadstena_libs_tilestorage_streams_hpp_included_
#define vadstena_libs_tilestorage_streams_hpp_included_

#include <ctime>
#include <iostream>
#include <memory>

#include <boost/noncopyable.hpp>
#include <boost/optional.hpp>
#include <boost/filesystem/path.hpp>

#include "./filetypes.hpp"

namespace vadstena { namespace tilestorage {

const char* contentType(File type);
const char* contentType(TileFile type);

struct FileStat {
    std::size_t size;
    std::time_t lastModified;
    const char *contentType;

    FileStat(std::size_t size, std::time_t lastModified
             , const char *contentType = "application/octet-stream")
        : size(size), lastModified(lastModified), contentType(contentType)
    {}

    static FileStat stat(const boost::filesystem::path &path);

    static FileStat stat(int fd);
};

struct ReadOnlyFd {
    int fd;
    std::size_t start;
    std::size_t end;

    ReadOnlyFd(int fd, std::size_t start, std::size_t end)
        : fd(fd), start(start), end(end)
    {}
};

class StreamBase : boost::noncopyable {
public:
    StreamBase(const char *contentType) : contentType_(contentType) {}
    StreamBase(File type) : contentType_(contentType(type)) {}
    StreamBase(TileFile type) : contentType_(contentType(type)) {}
    virtual ~StreamBase() {}

    FileStat stat() const;

    virtual FileStat stat_impl() const = 0;
    virtual void close() = 0;
    virtual std::string name() const = 0;

private:
    const char *contentType_;
};

class OStream : public StreamBase {
public:
    typedef std::shared_ptr<OStream> pointer;

    template <typename T>
    OStream(T &&value) : StreamBase(value) {}

    virtual std::ostream& get() = 0;

    operator std::ostream&() { return get(); }
};

class IStream : public StreamBase {
public:
    typedef std::shared_ptr<IStream> pointer;

    template <typename T>
    IStream(T &&value) : StreamBase(value) {}

    virtual std::istream& get() = 0;

    /** Read data from stream at given location.
     *  defaults to seek & read
     */
    virtual std::size_t read(char *buf, std::size_t size
                             , std::istream::pos_type off);

    operator std::istream&() { return get(); }

    /** Returns file descriptor associated with this stream. Returns boost::none
     *  if it is not possible to obtain such file descriptor (for example,
     *  stream is not associated with real file but resides in memory, etc.)
     */
    virtual boost::optional<ReadOnlyFd> fd() { return {}; }
};

void copyFile(const IStream::pointer &in, const OStream::pointer &out);

// inlines
inline FileStat StreamBase::stat() const
{
    auto stat(stat_impl());
    stat.contentType = contentType_;
    return stat;
}

} } // namespace vadstena::tilestorage

#endif // vadstena_libs_tilestorage_streams_hpp_included_

