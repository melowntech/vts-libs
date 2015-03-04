#include <cerrno>
#include <stdexcept>
#include <system_error>
#include <algorithm>

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

#include "utility/filedes.hpp"

#include "./tilar.hpp"

namespace vadstena { namespace tilestorage {

namespace fs = boost::filesystem;
using utility::Filedes;

struct Tilar::Detail
{
    Detail(const fs::path &path, const Options &options, Filedes &&fd)
        : path(path), options(options), fd(std::move(fd))
    {}

    const fs::path path;
    const Options options;
    Filedes fd;
};

Tilar::Tilar(Tilar::Detail *detail)
    : detail_(detail)
{}

Tilar::Tilar(Tilar &&o)
    : detail_(std::move(o.detail_))
{}

Tilar& Tilar::operator=(Tilar &&o)
{
    if (&o != this) {
        detail_ = std::move(o.detail_);
    }
    return *this;
}

Tilar::~Tilar() {}

namespace {

int flags(Tilar::OpenMode openMode)
{
    return ((openMode == Tilar::OpenMode::readOnly)
            ? O_RDONLY : O_RDWR);
}

int flags(Tilar::CreateMode createMode)
{
    switch (createMode) {
    case Tilar::CreateMode::failIfExists:
        return O_RDWR | O_EXCL | O_CREAT;

    case Tilar::CreateMode::truncate:
        return O_RDWR | O_TRUNC | O_CREAT;

    case Tilar::CreateMode::append:
        return O_RDWR | O_CREAT;
    }
    throw;
}

namespace constants {
    const std::size_t headerSize(8);
    const std::array<std::uint8_t, 5> magic({{ 'T', 'I', 'L', 'A', 'R' }});
    const std::uint8_t version(0);

    namespace index {
        const int version(5);
        const int binaryOrder(6);
        const int filesPerTile(7);
    }  // namespace index
} // namespace constants

off_t fileSize(const Filedes &fd)
{
    struct ::stat buf;
    if (-1 == ::fstat(fd, &buf)) {
        std::system_error e(errno, std::system_category());
        LOG(warn1)
            << "Failed to stat tilar file " << fd.path() << ": <" << e.code()
            << ", " << e.what() << ">.";
        throw e;
    }
    return buf.st_size;
}

Tilar::Options loadHeader(const Filedes &fd)
{
    std::array<std::uint8_t, constants::headerSize> header;

    auto left(header.size());
    unsigned char *data(header.data());
    while (left) {
        auto bytes(::read(fd, data, left));
        if (-1 == bytes) {
            if (EINTR == errno) { continue; }
            std::system_error e(errno, std::system_category());
            LOG(warn1)
                << "Unable to read from tilar file " << fd.path()
                << ": <" << e.code() << ", " << e.what() << ">.";
            throw e;
        }
        left -= bytes;
        data += left;
    }

    if (!std::equal(constants::magic.begin(), constants::magic.end()
                    , header.begin()))
    {
        LOGTHROW(warn1, std::runtime_error)
            << "Invalid magic in header of tilar file "
            << fd.path() << ".";
    }

    if (header[constants::index::version] != constants::version) {
        LOGTHROW(warn1, std::runtime_error)
            << "Invalid version in header of tilar file "
            << fd.path() << ": " << int(header[constants::index::version])
            << ".";
    }

    return { header[constants::index::binaryOrder]
            , header[constants::index::filesPerTile] };
}

void saveHeader(const Filedes &fd, const Tilar::Options &options)
{
    std::array<std::uint8_t, constants::headerSize> header;
    std::copy(constants::magic.begin(), constants::magic.end()
              , header.begin());
    header[constants::index::version] = constants::version;

    header[constants::index::binaryOrder] = options.binaryOrder;
    header[constants::index::filesPerTile] = options.filesPerTile;

    auto left(header.size());
    const unsigned char *data(header.data());
    while (left) {
        auto bytes(::write(fd, data, left));
        if (-1 == bytes) {
            if (EINTR == errno) { continue; }
            std::system_error e(errno, std::system_category());
            LOG(warn1)
                << "Unable to write to tilar file " << fd.path()
                << ": <" << e.code() << ", " << e.what() << ">.";
            throw e;
        }
        left -= bytes;
        data += left;
    }
}

} // namespace

Tilar Tilar::open(const fs::path &path, OpenMode openMode)
{
    Filedes fd(::open(path.string().c_str(), flags(openMode)));
    if (-1 == fd) {
        std::system_error e(errno, std::system_category());
        LOG(warn1)
            << "Failed to open tilar file " << path << ": <" << e.code()
            << ", " << e.what() << ">.";
        throw e;
    }

    return { new Detail(path, loadHeader(fd), std::move(fd)) };
}


Tilar Tilar::create(const fs::path &path, const Options &options
                    , CreateMode createMode)
{
    Filedes fd
        (::open(path.string().c_str(), flags(createMode)
                , S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP | S_IROTH | S_IWOTH)
         , path);
    if (-1 == fd) {
        std::system_error e(errno, std::system_category());
        LOG(warn1)
            << "Failed to create tilar file " << path << ": <" << e.code()
            << ", " << e.what() << ">.";
        throw e;
    }

    if (createMode != CreateMode::append) {
        // empty file -> write header
        saveHeader(fd, options);
    } else {
        auto size(fileSize(fd));
        if (!size) {
            // empty -> write header
            saveHeader(fd, options);
        } else {
            const auto current(loadHeader(fd));
            if (current != options) {
                LOGTHROW(warn1, std::runtime_error)
                    << "Unable to append file " << path
                    << ": file has different configuration.";
            }
        }
    }

    return { new Detail(path, options, std::move(fd)) };
}

} } // namespace vadstena::tilestorage
