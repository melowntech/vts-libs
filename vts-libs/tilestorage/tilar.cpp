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
#include "./error.hpp"

namespace vadstena { namespace tilestorage {

namespace fs = boost::filesystem;
using utility::Filedes;

namespace {

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

void truncate(const Filedes &fd, off_t size)
{
    if (-1 == ::ftruncate(fd, size)) {
        std::system_error e(errno, std::system_category());
        LOG(warn1)
            << "Failed to truncate file " << fd.path() << ": <" << e.code()
            << ", " << e.what() << ">.";
        throw e;
    }
}

} // namespace

struct Tilar::Detail
{
    Detail(const Options &options, Filedes &&fd, bool readOnly)
        : options(options), fd(std::move(fd)), readOnly(readOnly)
        , checkpoint(fileSize(this->fd)), tx(0)
    {}

    ~Detail();

    void flush();

    void wannaWrite(const std::string &what) const {
        if (readOnly) {
            LOGTHROW(err2, ReadOnlyError)
                << "Cannot " << what << ": archive " << fd.path()
                << " is read-only.";
        }
    }

    void begin(const Index &index, off_t start) {
        wannaWrite("start a transaction");
        if (tx) {
            LOGTHROW(err2, PendingTransaction)
                << "Pending transaction in archive " << fd.path() << ".";
        }
        txIndex = index;
        tx = start;
    }

    void rollback() {
        wannaWrite("rollback a transaction");
        if (!tx) {
            LOGTHROW(err2, PendingTransaction)
                << "No pending transaction in archive " << fd.path() << ".";
        }
        truncate(fd, tx);
        tx = 0;
    }

    void commit(off_t end) {
        wannaWrite("commit a transaction");
        if (!tx) {
            LOGTHROW(err2, PendingTransaction)
                << "No pending transaction in archive " << fd.path() << ".";
        }
        // TODO: update index
        (void) end;

        tx = 0;

        LOG(info4) << "Committed";
    }

    const Options options;
    Filedes fd;
    bool readOnly;

    /** End of file when this file was opened.
     */
    off_t checkpoint;

    Index txIndex;
    off_t tx;
};

void Tilar::Detail::flush()
{
    if (!readOnly) {
        // TODO: dump new index
        checkpoint = fileSize(fd);
    }
}

Tilar::Detail::~Detail()
{
    if (readOnly) { return; }

    if (fileSize(fd) != checkpoint) {
        // unflushed -> rollback
        LOG(warn2) << "File was not flushed, rolling back.";
        truncate(fd, checkpoint);
    }
}

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

    return { new Detail(loadHeader(fd), std::move(fd)
                        , (openMode == OpenMode::readOnly)) };
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

    return { new Detail(options, std::move(fd), false) };
}

namespace {

off_t seekFromEnd(const Filedes &fd, off_t pos = 0)
{
    const auto p(::lseek(fd, pos, SEEK_END));
    if (-1 == p) {
        std::system_error e(errno, std::system_category());
        LOG(warn1)
            << "Failed to seek to the end in the  tilar file "
            << fd.path() << ": <" << e.code()
            << ", " << e.what() << ">.";
        throw e;
    }
    return p;
}

} // namespace


class Tilar::Device {
public:
    typedef char char_type;

    // write constructor
    Device(Tilar::Detail &owner, const Index &index
           , off_t start)
        : owner(owner), fd(owner.fd), index(index)
        , start(start), pos(start), end(0)
    {
        owner.begin(index, start);
    }

    ~Device() {
        if (end || (pos == start)) { return; }
        try {
            if (!std::uncaught_exception()) {
                LOG(warn3) << "File write was not finished!";
            }
            LOG(warn1) << "Rolling back file.";
            owner.rollback();
        } catch (...) {}
    }

    void commit() {
        if (start != pos) {
            // commit transaction
            owner.commit(pos);
            // pretend there was nothing written
            start = pos;
        }
    }

    Tilar::Detail &owner;
    Filedes &fd;
    const Index index;

    off_t start;
    off_t pos;
    off_t end;
};

class Tilar::Sink {
public:
    typedef char char_type;
    typedef boost::iostreams::sink_tag category;

    Sink(Tilar::Detail &owner, const Index &index)
        : device_(std::make_shared<Device>
                  (owner, index, seekFromEnd(owner.fd)))
    {}


    void commit() { device_->commit(); }

    std::streamsize write(const char *s, std::streamsize n);

    class Stream;

private:
    std::shared_ptr<Device> device_;
};

class Tilar::Source {
public:
    typedef char char_type;
    typedef boost::iostreams::source_tag category;

    Source(Tilar::Detail &owner, const Index &index) {
        (void) owner; (void) index;
    }
        // : Device(owner, index) {}

    std::streamsize read(char *s, std::streamsize n);

    class Stream;
};

class Tilar::Sink::Stream
    : private boost::iostreams::stream_buffer<Tilar::Sink>
    , public tilestorage::OStream
{
public:
    Stream(Tilar::Detail &owner, const Index &index)
        : buffer_(owner, index), stream_(&buffer_)
    {}

    virtual std::ostream& get() { return stream_; }
    virtual void close() {
        stream_.flush();
        buffer_->commit();
        buffer_.close();
    }
    virtual std::string name() const { return "unknown"; }

private:
    boost::iostreams::stream_buffer<Tilar::Sink> buffer_;
    std::ostream stream_;
};

class Tilar::Source::Stream
    : private boost::iostreams::stream_buffer<Tilar::Sink>
    , public tilestorage::IStream
{
public:
    Stream(Tilar::Detail &owner, const Index &index)
        : boost::iostreams::stream_buffer<Tilar::Sink>(owner, index)
        , stream_(this)
    {}

    virtual std::istream& get() { return stream_; }
    virtual void close() {}
    virtual std::string name() const { return "unknown"; }

private:
    std::istream stream_;
};

std::streamsize Tilar::Sink::write(const char *data, std::streamsize size)
{
    auto &pos(device_->pos);
    const auto &fd(device_->fd);
    for (;;) {
        auto bytes(::pwrite(fd, data, size, pos));
        if (-1 == bytes) {
            if (EINTR == errno) { continue; }
            std::system_error e(errno, std::system_category());
            LOG(warn1)
                << "Unable to write to tilar file " << fd.path()
                << ": <" << e.code() << ", " << e.what() << ">.";
            throw e;
        }
        pos += bytes;
        return bytes;
    }
}

std::streamsize Tilar::Source::read(char *s, std::streamsize n)
{
    (void) s;
    return n;
}

void Tilar::flush()
{
    detail().flush();
}

OStream::pointer Tilar::output(const Index &index)
{
    return std::make_shared<Sink::Stream>(detail(), index);
}

IStream::pointer Tilar::input(const Index &index)
{
    return std::make_shared<Source::Stream>(detail(), index);
}

} } // namespace vadstena::tilestorage
