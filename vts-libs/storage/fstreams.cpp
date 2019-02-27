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
#include <iostream>
#include <functional>

#include <boost/filesystem.hpp>
#include <boost/iostreams/stream_buffer.hpp>
#include <boost/iostreams/device/file_descriptor.hpp>

#include "dbglog/dbglog.hpp"

#include "utility/gccversion.hpp"
#include "utility/streams.hpp"

#include "streams.hpp"
#include "error.hpp"

#include "fstreams.hpp"

namespace bio = boost::iostreams;

namespace vtslibs { namespace storage {

namespace detail {

const std::streamsize IOBufferSize = 1 << 16;

class FileOStream
    : public storage::OStream
{
public:
    typedef std::function<void(bool)> OnClose;

    template <typename Type>
    FileOStream(Type type, const boost::filesystem::path &path
                , OnClose onClose)
        : OStream(type), path_(path)
        , stream_(&buffer_)
        , onClose_(onClose)
    {
        stream_.exceptions(std::ios::badbit | std::ios::failbit);
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
            buffer_.open(bio::file_descriptor_sink
                         (path_.string()
                          , std::ios_base::out | std::ios_base::trunc)
                         , IOBufferSize);
        } catch (const std::exception &e) {
            LOGTHROW(err1, std::runtime_error)
                << "Unable to open file " << path_ << " for writing: <"
                << e.what() << ">.";
        }
    }

    boost::filesystem::path path_;
    bio::stream_buffer<bio::file_descriptor_sink> buffer_;
    std::ostream stream_;
    OnClose onClose_;
};

class FileIStream
    : public storage::IStream
{
public:
    template <typename Type>
    FileIStream(Type type, const boost::filesystem::path &path)
        : IStream(type), path_(path)
        , stream_(&buffer_)
    {
        stream_.exceptions(std::ios::badbit | std::ios::failbit);
        open();
    }

    // file doesn't need to be closed
    virtual ~FileIStream() {}

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

#if _WIN32
    // cannot convert from HANDLE to filedescriptor (only vice-versa)
    virtual boost::optional<ReadOnlyFd> fd() UTILITY_OVERRIDE {
        return boost::none;
    }

#else
    virtual boost::optional<ReadOnlyFd> fd() UTILITY_OVERRIDE {
        return ReadOnlyFd(buffer_->handle(), 0, stat_impl().size);
    }
#endif

private:
    void open() {
        try {
            stream_.exceptions(std::ios::badbit | std::ios::failbit);
            buffer_.open
                (bio::file_descriptor_source(path_.string())
                 , IOBufferSize);
        } catch (const std::exception &e) {
            if (!exists(path_)) {
                LOGTHROW(err1, NoSuchFile)
                    << "Unable to open file " << path_ << " for reading.";
            }
            LOGTHROW(err1, std::runtime_error)
                << "Unable to open file " << path_ << " for reading: <"
                << e.what() << ">.";
        }
    }

    boost::filesystem::path path_;
    bio::stream_buffer<bio::file_descriptor_source> buffer_;
    std::istream stream_;
};

} // namespace detail

OStream::pointer fileOStream(const char *contentType
                             , const boost::filesystem::path &path
                             , OnClose onClose)
{
    return std::make_shared<detail::FileOStream>(contentType, path, onClose);
}

OStream::pointer fileOStream(File type, const boost::filesystem::path &path
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

IStream::pointer fileIStream(File type, const boost::filesystem::path &path)
{
    return std::make_shared<detail::FileIStream>(type, path);
}

IStream::pointer fileIStream(File type, const boost::filesystem::path &path
                             , const NullWhenNotFound_t&)
{
    if (!exists(path)) { return {}; }
    return std::make_shared<detail::FileIStream>(type, path);
}

IStream::pointer fileIStream(TileFile type
                             , const boost::filesystem::path &path)

{
    return std::make_shared<detail::FileIStream>(type, path);
}

IStream::pointer fileIStream(TileFile type, const boost::filesystem::path &path
                             , const NullWhenNotFound_t&)

{
    if (!exists(path)) { return {}; }
    return std::make_shared<detail::FileIStream>(type, path);
}

} } // namespace vtslibs::storage
