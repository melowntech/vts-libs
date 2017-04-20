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
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>

#include <cerrno>
#include <system_error>

#include "dbglog/dbglog.hpp"

#include "utility/raise.hpp"

#include "./streams.hpp"
#include "./error.hpp"

namespace vtslibs { namespace storage {

const NullWhenNotFound_t NullWhenNotFound;

void copyFile(const IStream::pointer &in
              , const OStream::pointer &out)
{
    out->get() << in->get().rdbuf();
    in->close();
    out->close();
}

void copyFile(std::istream &in, const OStream::pointer &out)
{
    out->get() << in.rdbuf();
    out->close();
}

void copyFile(const IStream::pointer &in, std::ostream &out)
{
    out << in->get().rdbuf();
    in->close();
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

FileStat FileStat::stat(const boost::filesystem::path &path
                        , std::nothrow_t)
{
    struct ::stat st;
    if (::stat(path.string().c_str(), &st) == -1) {
        return { 0, 0 };
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

FileStat FileStat::stat(int fd, std::nothrow_t)
{
    struct ::stat st;
    if (::fstat(fd, &st) == -1) {
        return { 0, 0 };
    }

    return { std::size_t(st.st_size), st.st_mtime
            , "application/octet-stream" };
}

const char* contentType(File type)
{
    switch (type) {
    case File::config:
    case File::registry:
        return "application/json; charset=utf-8";

    default: break;
    }
    return "application/octet-stream";
}

const char* contentType(TileFile type)
{
    switch (type) {
    case TileFile::atlas:
    case TileFile::navtile:
        return "image/jpeg";

    case TileFile::meta2d:
    case TileFile::mask:
        return "image/png";

    case TileFile::credits:
        return "application/json; charset=utf-8";

    default: break;
    }
    return "application/octet-stream";
}

bool gzipped(std::istream &s)
{
    return (s.peek() == 0x1f);
}

bool gzipped(const IStream::pointer &s)
{
    return (gzipped(*s));
}

bool gzipped(std::istream &s, std::size_t offset)
{
    s.seekg(offset);
    return gzipped(s);
}

bool gzipped(const IStream::pointer &s, std::size_t offset)
{
    return gzipped(*s, offset);
}

} } // namespace vtslibs::storage

