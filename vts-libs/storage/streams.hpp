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
#ifndef vtslibs_storage_streams_hpp_included_
#define vtslibs_storage_streams_hpp_included_

#include <ctime>
#include <iostream>
#include <sstream>
#include <memory>
#include <new>

#include <boost/noncopyable.hpp>
#include <boost/optional.hpp>
#include <boost/filesystem/path.hpp>

#ifdef _WIN32
#  ifndef WIN32_LEAN_AND_MEAN
#    define WIN32_LEAN_AND_MEAN
#  endif
#  include <Windows.h>
#endif

#include "filetypes.hpp"

namespace vtslibs { namespace storage {

const char* contentType(File type);
const char* contentType(TileFile type);

struct NullWhenNotFound_t {};
const extern NullWhenNotFound_t NullWhenNotFound;

struct FileStat {
    std::size_t size;
    std::time_t lastModified;
    const char *contentType;

    FileStat(std::size_t size = 0, std::time_t lastModified = 0
             , const char *contentType = "application/octet-stream")
        : size(size), lastModified(lastModified), contentType(contentType)
    {}

    bool changed(const FileStat &other) const;

    static FileStat stat(const boost::filesystem::path &path);
    static FileStat stat(const boost::filesystem::path &path
                         , std::nothrow_t);

    static FileStat stat(int fd);
    static FileStat stat(int fd, std::nothrow_t);
#ifdef _WIN32
    static FileStat stat(::HANDLE h);
    static FileStat stat(::HANDLE h, std::nothrow_t);
#endif
};

struct PathStat : FileStat {
    boost::filesystem::path path;

    typedef std::vector<PathStat> list;

    PathStat(const boost::filesystem::path &path)
        : FileStat(PathStat::stat(path))
        , path(path)
    {}

    bool changed() const {
        return FileStat::changed(PathStat::stat(path, std::nothrow));
    }
};

bool changed(const PathStat::list &stats);

/** Read only file descriptor.
 *  Can be returned by IStream to access data directly.
 *
 *  If shared is true then file position cannot be changed because there can be
 *  more streams sharing the same file descriptor -- pread must be used instead
 *  of read and seek is forbidden.
 */
struct ReadOnlyFd {
    int fd;
    std::size_t start;
    std::size_t end;
    bool shared;

    ReadOnlyFd(int fd, std::size_t start, std::size_t end
               , bool shared = false)
        : fd(fd), start(start), end(end), shared(shared)
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

protected:
    const char *contentType_;
};

class OStream
    : public StreamBase
    , public std::enable_shared_from_this<OStream>
{
public:
    typedef std::shared_ptr<OStream> pointer;

    template <typename T>
    OStream(T &&value) : StreamBase(value) {}

    virtual std::ostream& get() = 0;

    operator std::ostream&() { return get(); }
};

class IStream
    : public StreamBase
    , public std::enable_shared_from_this<IStream>
{
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

/** Special in-memory stream.
 */
class StringIStream : public IStream {
public:
    StringIStream(TileFile type, const std::string &name
                  , std::time_t lastModified)
        : IStream(type), stat_(0, lastModified), name_(name)
    {}

    StringIStream(File type, const std::string &name
                  , std::time_t lastModified)
        : IStream(type), stat_(0, lastModified), name_(name)
    {}

    StringIStream(const IStream &is)
        : IStream(is.stat().contentType)
        , stat_(0, is.stat().lastModified)
        , name_(is.name())
    {}

    std::stringstream& sink() { return ss_; }

    void updateSize() { stat_.size = ss_.tellp(); }

private:
    virtual std::istream& get() { return ss_; }
    virtual FileStat stat_impl() const { return stat_; }
    virtual void close() {};
    virtual std::string name() const { return name_; }

    FileStat stat_;
    std::string name_;
    std::stringstream ss_;
};

void copyFile(const IStream::pointer &in, const OStream::pointer &out);
void copyFile(std::istream &in, const OStream::pointer &out);
void copyFile(const IStream::pointer &in, std::ostream &out);

/** Tells whether stream is gzipped. Doesn't seek nor read anything.
 */
bool gzipped(std::istream &s);

/** Tells whether stream is gzipped. Doesn't seek nor read anything.
 */
bool gzipped(const IStream::pointer &s);

/** Tells whether stream is gzipped at given position.
 *  Seeks at given position but does not read anything.
 */
bool gzipped(std::istream &s, std::size_t offset);

/** Tells whether stream is gzipped at given position.
 *  Seeks at given position but does not read anything.
 */
bool gzipped(const IStream::pointer &s, std::size_t offset);

// inlines
inline FileStat StreamBase::stat() const
{
    auto stat(stat_impl());
    stat.contentType = contentType_;
    return stat;
}

inline bool FileStat::changed(const FileStat &other) const
{
    return (size != other.size) || (lastModified != other.lastModified);
}

} } // namespace vtslibs::storage

#endif // vtslibs_storage_streams_hpp_included_

