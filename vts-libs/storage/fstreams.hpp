#ifndef vadstena_libs_storage_driver_fstreams_hpp_included_
#define vadstena_libs_storage_driver_fstreams_hpp_included_

#include <functional>

#include <boost/filesystem.hpp>

#include "./streams.hpp"
#include "./error.hpp"

namespace vadstena { namespace storage {

typedef std::function<void(bool)> OnClose;

OStream::pointer fileOStream(const char *contentType
                             , const boost::filesystem::path &path
                             , OnClose onClose = OnClose());
OStream::pointer fileOStream(File type
                             , const boost::filesystem::path &path
                             , OnClose onClose = OnClose());
OStream::pointer fileOStream(TileFile type
                             , const boost::filesystem::path &path
                             , OnClose onClose = OnClose());

IStream::pointer fileIStream(const char *contentType
                             , const boost::filesystem::path &path);
IStream::pointer fileIStream(File type
                             , const boost::filesystem::path &path);
IStream::pointer fileIStream(File type
                             , const boost::filesystem::path &path
                             , const NullWhenNotFound_t&);
IStream::pointer fileIStream(TileFile type
                             , const boost::filesystem::path &path);
IStream::pointer fileIStream(TileFile type
                             , const boost::filesystem::path &path
                             , const NullWhenNotFound_t&);

} } // namespace vadstena::storage

#endif // vadstena_libs_storage_driver_fstreams_hpp_included_
