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
#ifndef vtslibs_storage_driver_sstreams_hpp_included_
#define vtslibs_storage_driver_sstreams_hpp_included_

#include <functional>

#include <boost/filesystem.hpp>

#include "streams.hpp"
#include "error.hpp"

namespace vtslibs { namespace storage {

typedef std::vector<char> MemBlock;

IStream::pointer memIStream(const char *contentType, std::string &&data
                            , std::time_t lastModified = 0
                            , const boost::filesystem::path &path = "unknown");
IStream::pointer memIStream(const char *contentType, const std::string &data
                            , std::time_t lastModified = 0
                            , const boost::filesystem::path &path = "unknown");

IStream::pointer memIStream(File type, const std::string &data
                            , std::time_t lastModified = 0
                            , const boost::filesystem::path &path = "unknown");
IStream::pointer memIStream(File type, std::string &&data
                            , std::time_t lastModified = 0
                            , const boost::filesystem::path &path = "unknown");

IStream::pointer memIStream(TileFile type, const std::string &data
                            , std::time_t lastModified = 0
                            , const boost::filesystem::path &path = "unknown");
IStream::pointer memIStream(TileFile type, std::string &&data
                            , std::time_t lastModified = 0
                            , const boost::filesystem::path &path = "unknown");

IStream::pointer memIStream(const char *contentType, MemBlock &&data
                            , std::time_t lastModified = 0
                            , const boost::filesystem::path &path = "unknown");
IStream::pointer memIStream(const char *contentType, const MemBlock &data
                            , std::time_t lastModified = 0
                            , const boost::filesystem::path &path = "unknown");

IStream::pointer memIStream(File type, const MemBlock &data
                            , std::time_t lastModified = 0
                            , const boost::filesystem::path &path = "unknown");
IStream::pointer memIStream(File type, MemBlock &&data
                            , std::time_t lastModified = 0
                            , const boost::filesystem::path &path = "unknown");
IStream::pointer memIStream(TileFile type, const MemBlock &data
                            , std::time_t lastModified = 0
                            , const boost::filesystem::path &path = "unknown");
IStream::pointer memIStream(TileFile type, MemBlock &&data
                            , std::time_t lastModified = 0
                            , const boost::filesystem::path &path = "unknown");

} } // namespace vtslibs::storage

#endif // vtslibs_storage_driver_sstreams_hpp_included_
