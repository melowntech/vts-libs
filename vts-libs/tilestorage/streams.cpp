#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>

#include <cerrno>
#include <system_error>

#include "dbglog/dbglog.hpp"

#include "utility/raise.hpp"

#include "./streams.hpp"

namespace vadstena { namespace tilestorage {

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

} } // namespace vadstena::tilestorage

