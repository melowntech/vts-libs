#ifndef vtslibs_storage_driver_sstreams_hpp_included_
#define vtslibs_storage_driver_sstreams_hpp_included_

#include <functional>

#include <boost/filesystem.hpp>

#include "./streams.hpp"
#include "./error.hpp"

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
