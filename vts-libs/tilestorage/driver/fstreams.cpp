#include <iostream>
#include <functional>

#include <boost/filesystem.hpp>
#include <boost/iostreams/stream_buffer.hpp>
#include <boost/iostreams/device/file_descriptor.hpp>

#include "dbglog/dbglog.hpp"

#include "utility/gccversion.hpp"
#include "utility/streams.hpp"

#include "../streams.hpp"
#include "../error.hpp"

#include "./fstreams.hpp"

namespace vadstena { namespace tilestorage {

namespace detail {

const std::streamsize IOBufferSize = 1 << 16;

class FileOStream
    : public tilestorage::OStream
{
public:
    typedef std::function<void(bool)> OnClose;

    template <typename Type>
    FileOStream(Type type, const boost::filesystem::path &path
                , OnClose onClose)
        : OStream(type), path_(path), stream_(&buffer_), onClose_(onClose)
    {
        buf_.reset(new char[IOBufferSize]);
        buffer_.pubsetbuf(buf_.get(), IOBufferSize);
        open();
    }

    virtual ~FileOStream() {
        if (!std::uncaught_exception() && buffer_.is_open()) {
            LOG(warn3) << "File was not closed!";
        }

        try {
            if (buffer_.is_open() && onClose_) {
                onClose_(false);
            }
        } catch (...) {}
    }

    virtual std::ostream& get() UTILITY_OVERRIDE { return stream_; }

    virtual void close() UTILITY_OVERRIDE {
        // TODO: call onClose in case of failure (when exception is thrown)
        // via utility::ScopeGuard
        buffer_.close();
        if (onClose_) { onClose_(true); }
    }

    virtual std::string name() const UTILITY_OVERRIDE {
        return path_.string();
    };

    virtual FileStat stat_impl() const UTILITY_OVERRIDE {
        // stream_buffer has only non-const version of operator-> :(
        return FileStat::stat
            (const_cast<decltype(buffer_)&>(buffer_)->handle());
    }

private:
    void open() {
        try {
            stream_.exceptions(std::ios::badbit | std::ios::failbit);
            buffer_.open(path_.string()
                         , std::ios_base::out | std::ios_base::trunc);
        } catch (const std::exception &e) {
            LOGTHROW(err1, std::runtime_error)
                << "Unable to open file " << path_ << " for writing.";
        }
    }

    boost::filesystem::path path_;
    std::unique_ptr<char[]> buf_;
    boost::iostreams::stream_buffer
    <boost::iostreams::file_descriptor_sink> buffer_;
    std::ostream stream_;
    OnClose onClose_;
};

class FileIStream
    : public tilestorage::IStream
{
public:
    template <typename Type>
    FileIStream(Type type, const boost::filesystem::path &path)
        : IStream(type), path_(path), stream_(&buffer_)
    {
        buf_.reset(new char[IOBufferSize]);
        buffer_.pubsetbuf(buf_.get(), IOBufferSize);
        open();
    }

    virtual ~FileIStream() {
        if (!std::uncaught_exception() && buffer_.is_open()) {
            LOG(warn3) << "File was not closed!";
        }
    }

    virtual std::istream& get() UTILITY_OVERRIDE { return stream_; }

    virtual void close() UTILITY_OVERRIDE { buffer_.close(); }

    virtual std::string name() const UTILITY_OVERRIDE {
        return path_.string();
    };

    virtual FileStat stat_impl() const UTILITY_OVERRIDE {
        // stream_buffer has only non-const version of operator-> :(
        return FileStat::stat
            (const_cast<decltype(buffer_)&>(buffer_)->handle());
    }

    virtual boost::optional<ReadOnlyFd> fd() UTILITY_OVERRIDE {
        return ReadOnlyFd(buffer_->handle(), 0, stat_impl().size);
    }

private:
    void open() {
        try {
            stream_.exceptions(std::ios::badbit | std::ios::failbit);
            buffer_.open(path_.string());
        } catch (const std::exception &e) {
            if (!exists(path_)) {
                LOGTHROW(err1, NoSuchFile)
                    << "Unable to open file " << path_ << " for reading.";
            }
            LOGTHROW(err1, std::runtime_error)
                << "Unable to open file " << path_ << " for reading.";
        }
    }

    boost::filesystem::path path_;
    std::unique_ptr<char[]> buf_;
    boost::iostreams::stream_buffer
    <boost::iostreams::file_descriptor_source> buffer_;
    std::istream stream_;
};

} // namespace detail

OStream::pointer fileOStream(const char *contentType
                             , const boost::filesystem::path &path
                             , OnClose onClose)
{
    return std::make_shared<detail::FileOStream>(contentType, path, onClose);
}

OStream::pointer fileOStream(File type
                             , const boost::filesystem::path &path
                             , OnClose onClose)
{
    return std::make_shared<detail::FileOStream>(type, path, onClose);
}

OStream::pointer fileOStream(TileFile type
                             , const boost::filesystem::path &path
                             , OnClose onClose)
{
    return std::make_shared<detail::FileOStream>(type, path, onClose);
}

IStream::pointer fileIStream(const char *contentType
                             , const boost::filesystem::path &path)
{
    return std::make_shared<detail::FileIStream>(contentType, path);
}

IStream::pointer fileIStream(File type
                             , const boost::filesystem::path &path)
{
    return std::make_shared<detail::FileIStream>(type, path);
}

IStream::pointer fileIStream(TileFile type
                             , const boost::filesystem::path &path)

{
    return std::make_shared<detail::FileIStream>(type, path);
}

} } // namespace vadstena::tilestorage
