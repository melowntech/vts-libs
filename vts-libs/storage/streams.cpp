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

#include <cerrno>
#include <system_error>

#include "dbglog/dbglog.hpp"

#include "utility/raise.hpp"
#include "utility/filesystem.hpp"
#include "utility/time.hpp"

#include "streams.hpp"
#include "error.hpp"

namespace vtslibs { namespace storage {

const NullWhenNotFound_t NullWhenNotFound{};

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
    auto s(utility::FileStat::from(path));
    return { s.size, s.modified, "application/octet-stream" };
}

FileStat FileStat::stat(const boost::filesystem::path &path
                        , std::nothrow_t)
{
    auto s(utility::FileStat::from(path, std::nothrow));
    return { s.size, s.modified, "application/octet-stream" };
}

FileStat FileStat::stat(int fd)
{
    auto s(utility::FileStat::from(fd));
    return { s.size, s.modified, "application/octet-stream" };
}

FileStat FileStat::stat(int fd, std::nothrow_t)
{
    auto s(utility::FileStat::from(fd, std::nothrow));
    return { s.size, s.modified, "application/octet-stream" };
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

#ifdef _WIN32

FileStat FileStat::stat(::HANDLE h)
{
    ::LARGE_INTEGER size;
    if (!::GetFileSizeEx(h, &size)) {
        std::system_error e(::GetLastError(), std::generic_category()
                            , utility::formatError
                            ("Cannot get file size via handle %s.", h));
        LOG(err1) << e.what();
        throw e;
    }

    ::ULARGE_INTEGER modified;
    {
        ::FILETIME tmp;
        if (!::GetFileTime(h, nullptr, nullptr, &tmp)) {
            std::system_error e(::GetLastError(), std::generic_category()
                                , utility::formatError
                                ("Cannot get file time via handle %s.", h));
            LOG(err1) << e.what();
            throw e;
        }

        modified.LowPart  = tmp.dwLowDateTime;
        modified.HighPart = tmp.dwHighDateTime;
    }

    return { std::size_t(size.QuadPart)
            , utility::windowsFileTime2Unix(modified.QuadPart)
            , "application/octet-stream"};
}

FileStat FileStat::stat(::HANDLE h, std::nothrow_t)
{
    ::LARGE_INTEGER size;
    if (!::GetFileSizeEx(h, &size)) { return { 0, -1 }; }

    ::ULARGE_INTEGER modified;
    {
        ::FILETIME tmp;
        if (!::GetFileTime(h, nullptr, nullptr, &tmp)) { return { 0, -1 }; }

        modified.LowPart  = tmp.dwLowDateTime;
        modified.HighPart = tmp.dwHighDateTime;
    }

    return { std::size_t(size.QuadPart)
            , utility::windowsFileTime2Unix(modified.QuadPart)
            , "application/octet-stream"};
}

#endif

bool changed(const PathStat::list &stats)
{
    for (const auto &stat : stats) {
        if (stat.changed()) { return true; }
    }
    return false;
}


} } // namespace vtslibs::storage

