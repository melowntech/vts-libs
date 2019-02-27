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
#include <boost/iostreams/device/array.hpp>

#include "dbglog/dbglog.hpp"

#include "utility/gccversion.hpp"

#include "sstreams.hpp"

namespace bio = boost::iostreams;

namespace vtslibs { namespace storage {

namespace detail {

//const std::streamsize IOBufferSize = 1 << 16;

class MemIStream : public storage::IStream {
public:
    template <typename Type>
    MemIStream(Type type, std::time_t lastModified
               , const char *begin, const char *end
               , const boost::filesystem::path &path)
        : IStream(type)
        , path_(path)
        , buffer_(begin, end)
        , stream_(&buffer_)
        , fs_(end - begin, lastModified)
    {
        stream_.exceptions(std::ios::badbit | std::ios::failbit);
    }

    // file doesn't need to be closed
    virtual ~MemIStream() {}

    virtual std::istream& get() UTILITY_OVERRIDE { return stream_; }

    virtual void close() UTILITY_OVERRIDE { }

    virtual std::string name() const UTILITY_OVERRIDE {
        return path_.string();
    };

    virtual FileStat stat_impl() const UTILITY_OVERRIDE { return fs_; }

private:

    boost::filesystem::path path_;
    bio::stream_buffer<bio::array_source> buffer_;
    std::istream stream_;
    FileStat fs_;
};

template <typename Data>
struct MemHolder {
    MemHolder(Data &&data) : data(std::move(data)) {}
    Data data;
};

template <typename Data>
class MemIStreamWithHolder
    : private MemHolder<Data>
    , public MemIStream
{
public:
    template <typename Type>
    MemIStreamWithHolder(Type type, Data &&indata
                         , std::time_t lastModified
                         , const boost::filesystem::path &path)
        : MemHolder<Data>(std::move(indata))
        , MemIStream(type, lastModified, this->data.data()
                     , this->data.data() + this->data.size(), path)
    {}
};

template <typename Data, typename Type>
IStream::pointer memStream(Type type, Data &&indata
                           , std::time_t lastModified
                           , const boost::filesystem::path &path)
{
    return std::make_shared<detail::MemIStreamWithHolder<Data>>
        (type, std::move(indata), lastModified, path);
}

} // namespace detail

IStream::pointer memIStream(const char *contentType, std::string &&data
                            , std::time_t lastModified
                            , const boost::filesystem::path &path)
{
    return detail::memStream(contentType, std::move(data), lastModified, path);
}

IStream::pointer memIStream(const char *contentType, const std::string &data
                            , std::time_t lastModified
                            , const boost::filesystem::path &path)
{
    return detail::memStream(contentType, std::move(data), lastModified, path);
}

IStream::pointer memIStream(File type, const std::string &data
                            , std::time_t lastModified
                            , const boost::filesystem::path &path)
{
    return detail::memStream(type, std::move(data), lastModified, path);
}

IStream::pointer memIStream(File type, std::string &&data
                            , std::time_t lastModified
                            , const boost::filesystem::path &path)
{
    return detail::memStream(type, std::move(data), lastModified, path);
}

IStream::pointer memIStream(TileFile type, const std::string &data
                            , std::time_t lastModified
                            , const boost::filesystem::path &path)
{
    return detail::memStream(type, std::move(data), lastModified, path);
}

IStream::pointer memIStream(TileFile type, std::string &&data
                            , std::time_t lastModified
                            , const boost::filesystem::path &path)
{
    return detail::memStream(type, std::move(data), lastModified, path);
}

IStream::pointer memIStream(const char *contentType, MemBlock &&data
                            , std::time_t lastModified
                            , const boost::filesystem::path &path)
{
    return detail::memStream(contentType, std::move(data), lastModified, path);
}

IStream::pointer memIStream(const char *contentType, const MemBlock &data
                            , std::time_t lastModified
                            , const boost::filesystem::path &path)
{
    return detail::memStream(contentType, std::move(data), lastModified, path);
}

IStream::pointer memIStream(File type, const MemBlock &data
                            , std::time_t lastModified
                            , const boost::filesystem::path &path)
{
    return detail::memStream(type, std::move(data), lastModified, path);
}

IStream::pointer memIStream(File type, MemBlock &&data
                            , std::time_t lastModified
                            , const boost::filesystem::path &path)
{
    return detail::memStream(type, std::move(data), lastModified, path);
}

IStream::pointer memIStream(TileFile type, const MemBlock &data
                            , std::time_t lastModified
                            , const boost::filesystem::path &path)
{
    return detail::memStream(type, std::move(data), lastModified, path);
}

IStream::pointer memIStream(TileFile type, MemBlock &&data
                            , std::time_t lastModified
                            , const boost::filesystem::path &path)
{
    return detail::memStream(type, std::move(data), lastModified, path);
}

} } // namespace vtslibs::storage
