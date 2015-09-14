#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>

#include <cerrno>
#include <system_error>

#include "dbglog/dbglog.hpp"

#include "utility/raise.hpp"

#include "./streams.hpp"
#include "./error.hpp"

namespace vadstena { namespace storage {

void copyFile(const IStream::pointer &in
              , const OStream::pointer &out)
{
    out->get() << in->get().rdbuf();
    in->close();
    out->close();
}

std::size_t IStream::read(char *buf, std::size_t size
                          , std::istream::pos_type off)
{
    auto &s(get());
    s.seekg(off);
    if (s.tellg() != off) {
        // seek past end && read -> EOF
        return 0;
    }
    return s.read(buf, size).gcount();
}

FileStat FileStat::stat(const boost::filesystem::path &path)
{
    struct ::stat st;
    if (::stat(path.string().c_str(), &st) == -1) {
        std::system_error e
            (errno, std::system_category()
             , utility::formatError("Cannot stat file %s.", path));
        LOG(err1) << e.what();
        throw e;
    }

    return { std::size_t(st.st_size), st.st_mtime
            , "application/octet-stream" };
}

FileStat FileStat::stat(int fd)
{
    struct ::stat st;
    if (::fstat(fd, &st) == -1) {
        std::system_error e
            (errno, std::system_category()
             , utility::formatError("Cannot stat fd %d.", fd));
        LOG(err1) << e.what();
        throw e;
    }

    return { std::size_t(st.st_size), st.st_mtime
            , "application/octet-stream" };
}

const char* contentType(File type)
{
    if (type == File::config) {
        return "application/json";
    }
    return "application/octet-stream";
}

const char* contentType(TileFile type)
{
    if (type == TileFile::atlas) {
        return "image/jpeg";
    }
    return "application/octet-stream";
}

#if 0

namespace multi {

std::streamsize IOBufferSize(1 << 16);

class Sink {
public:
    typedef char char_type;
    typedef boost::iostreams::sink_tag category;

    Sink(const OStream::pointer &stream, off_t start)
        : stream_(stream), start_(start), pos(0)
    {}

    std::string name() const { return stream_->name(); }

    std::streamsize write(const char *s, std::streamsize n);

    FileStat stat() const;

    class Stream;

private:
    OStream::pointer stream_;
    off_t start_;
    off_t pos_;
};

class Sink::Stream
    : public OStream
{
public:
    Stream(const OStream::pointer &owner, const FileRange &range)
        , OStream(owner->contentType())
        , buffer_(owner, index), stream_(&buffer_)
    {
        stream_.exceptions(std::ios::badbit | std::ios::failbit);
        buf_.reset(new char[IOBufferSize]);
        buffer_.pubsetbuf(buf_.get(), IOBufferSize);
    }

    virtual std::ostream& get() UTILITY_OVERRIDE { return stream_; }
    virtual void close() UTILITY_OVERRIDE {
        stream_.flush();
        buffer_.close();
    }
    virtual std::string name() const UTILITY_OVERRIDE {
        // stream_buffer has only non-const version of operator-> :(
        return const_cast<decltype(buffer_)&>(buffer_)->name();
    }

    virtual FileStat stat_impl() const UTILITY_OVERRIDE {
        // stream_buffer has only non-const version of operator-> :(
        return const_cast<decltype(buffer_)&>(buffer_)->stat();
    }

private:
    std::unique_ptr<char[]> buf_;
    boost::iostreams::stream_buffer<Tilar::Sink> buffer_;
    std::ostream stream_;
};

class MultiIStream
    : public IStream
{
public:
    typedef std::shared_ptr<MultiIStream> pointer;

    MultiIStream(const IStream::pointer &stream)
        : IStream(stream->contentType())
        , stream_(stream)
    {}

    virtual std::istream& get() { return stream_->get(); }

    virtual std::size_t read(char *buf, std::size_t size
                             , std::istream::pos_type off)
    {
        return stream_->read(buf, size, off);
    }

    virtual boost::optional<ReadOnlyFd> fd() { return stream_->fd(); }

    virtual pointer sub(std::size_t index);

    virtual FileStat stat_impl();

    virtual void close() { return stream_->close(); }

    virtual std::string name() const { return stream_->name(); }

    virtual std::size_t subs() const { return 1; }

private:
    IStream::pointer stream_;
};

FileStat MultiIStream::stat_impl()
{
    // TODO: implement me
    return stream_->stat_impl();
}

} // namespace multi

IStream::pointer multiStream(const IStream::pointer &is)
{
    return std::make_shared<multi::MultiIStream>(is);
}

OStream::pointer multiStream(const OStream::pointer &os)
{
    return os;
}
#endif

} } // namespace vadstena::storage

