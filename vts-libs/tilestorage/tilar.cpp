#include <cerrno>
#include <ctime>
#include <stdexcept>
#include <system_error>
#include <algorithm>

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

#include <boost/crc.hpp>

#include "utility/filedes.hpp"

#include "./tilar.hpp"
#include "./error.hpp"

namespace vadstena { namespace tilestorage {

namespace fs = boost::filesystem;
using utility::Filedes;

namespace {

typedef std::uint8_t Version;

namespace header_constants {
    const std::size_t size(8);
    const std::array<std::uint8_t, 5> magic({{ 'T', 'I', 'L', 'A', 'R' }});
    const Version version(0);

    namespace index {
        const int version(5);
        const int binaryOrder(6);
        const int filesPerTile(7);
    }  // namespace index
} // namespace header

namespace index_constants {
    const std::size_t size(20);
    const std::array<std::uint8_t, 4> magic({{ 'T', 'I', 'D', 'X' }});

    namespace index {
        const int overhead(4);
        const int timestamp(8);
        const int crc32(16);
    }  // namespace index
} // namespace index_constants

template <typename T, size_t S>
void serialize(std::array<std::uint8_t, S> &out, int index, const T &value)
{
    *reinterpret_cast<T*>(out.data() + index) = value;
}

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

template <size_t S>
void write(const Filedes &fd, const std::array<std::uint8_t, S> &block)
{
    auto left(block.size());
    const unsigned char *data(block.data());
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

template <size_t S>
void read(const Filedes &fd, std::array<std::uint8_t, S> &block)
{
    auto left(block.size());
    unsigned char *data(block.data());
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
}

std::tuple<Tilar::Options, std::uint8_t> loadHeader(const Filedes &fd)
{
    std::array<std::uint8_t, header_constants::size> header;
    read(fd, header);

    if (!std::equal(header_constants::magic.begin()
                    , header_constants::magic.end()
                    , header.begin()))
    {
        LOGTHROW(warn1, std::runtime_error)
            << "Invalid magic in header of tilar file "
            << fd.path() << ".";
    }

    auto version(header[header_constants::index::version]);
    if (version != header_constants::version) {
        LOGTHROW(warn1, std::runtime_error)
            << "Invalid version in header of tilar file "
            << fd.path() << ": " << int(version) << ".";
    }

    return std::tuple<Tilar::Options, std::uint8_t>
        (Tilar::Options{ header[header_constants::index::binaryOrder]
                         , header[header_constants::index::filesPerTile] }
         , version);
}

Version saveHeader(const Filedes &fd, const Tilar::Options &options)
{
    auto version(header_constants::version);

    std::array<std::uint8_t, header_constants::size> header;
    std::copy(header_constants::magic.begin(), header_constants::magic.end()
              , header.begin());
    header[header_constants::index::version] = version;

    header[header_constants::index::binaryOrder] = options.binaryOrder;
    header[header_constants::index::filesPerTile] = options.filesPerTile;

    write(fd, header);

    return version;
}


inline unsigned int edge(const Tilar::Options &o) {
    return 1u << o.binaryOrder;
}

inline unsigned int tiles(const Tilar::Options &o) {
    return 1u << (2 * o.binaryOrder);
}
inline unsigned int files(const Tilar::Options &o) {
    return o.filesPerTile * tiles(o);
}

class ArchiveIndex {
public:
    struct Slot {
        std::uint32_t start;
        std::uint32_t size;

        Slot() : start(0), size(0) {}
    };

    ArchiveIndex(const Tilar::Options &options)
        : options_(options_)
        , rowSkip_(sizeof(Slot) * edge(options))
        , gridSkip_(sizeof(Slot) * tiles(options))
        , grids_(files(options), Slot())
        , overhead_(0)
    {}

    /** Loads index from given file descriptor. If (pos <= 0) then the search
     *  for Spock^Wthe index is initiated from the current end of the file.
     */
    void load(const Filedes &fd, off_t pos = 0);

    /** Saves new index to the end of the file;
     */
    void save(const Filedes &fd) const;

    void set(const Tilar::FileIndex &index, off_t start, off_t end);

    void unset(const Tilar::FileIndex &index);

    std::uint32_t crc(std::uint64_t timestamp) const {
        boost::crc_32_type crc;
        crc.process_bytes(&overhead_, sizeof(overhead_));
        crc.process_bytes(&timestamp, sizeof(timestamp));
        crc.process_bytes(grids_.data(), grids_.size());
        return crc.checksum();
    }

private:
    inline Slot& slot(const Tilar::FileIndex &index) {
        return grids_[index.col
                      + (rowSkip_ * index.row)
                      + (gridSkip_ * index.type)];
    }

    inline const Slot& slot(const Tilar::FileIndex &index) const {
        return grids_[index.col
                      + (rowSkip_ * index.row)
                      + (gridSkip_ * index.type)];
    }

    typedef std::vector<Slot> Grids;

    const Tilar::Options options_;
    const int rowSkip_;
    const int gridSkip_;
    Grids grids_;
    std::uint32_t overhead_;
};

void ArchiveIndex::save(const Filedes &fd) const
{
    LOG(info1) << "Saving new archive index.";
    std::uint64_t timestamp(std::time(nullptr));

    // save header first
    std::array<std::uint8_t, index_constants::size> header;
    std::copy(index_constants::magic.begin(), index_constants::magic.end()
              , header.begin());

    serialize(header, index_constants::index::overhead, overhead_);
    serialize(header, index_constants::index::timestamp, timestamp);
    serialize(header, index_constants::index::crc32, crc(timestamp));

    seekFromEnd(fd);
    write(fd, header);
}

void ArchiveIndex::set(const Tilar::FileIndex &index, off_t start, off_t end)
{
    const auto size(end - start);
    auto &s(slot(index));

    if (s.start) {
        overhead_ += size;
    }
    s.start = start;
    s.size = size;
}

void ArchiveIndex::unset(const Tilar::FileIndex &index)
{
    auto &s(slot(index));

    if (s.start) {
        overhead_ += s.size;
    }
    s.start = s.size = 0;
}

} // namespace

struct Tilar::Detail
{
    Detail(const Detail&) = delete;
    Detail& operator=(const Detail&) = delete;
    Detail(Detail&&) = delete;
    Detail& operator=(Detail&&) = delete;

    Detail(std::uint8_t version, const Options &options
           , Filedes &&fd, bool readOnly)
        : version(version), options(options), fd(std::move(fd))
        , readOnly(readOnly), index(options)
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
        LOG(debug) << fd.path() << ": " <<  what << ".";
    }

    void begin(const FileIndex &index, off_t start) {
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

        // update index slot and forget transaction
        index.set(txIndex, tx, end);
        tx = 0;
    }

    Version version;
    const Options options;
    Filedes fd;
    bool readOnly;

    ArchiveIndex index;

    /** End of file when this file was opened.
     */
    off_t checkpoint;

    FileIndex txIndex;
    off_t tx;
};

void Tilar::Detail::flush()
{
    if (!readOnly) {
        // save index and remember new checkpoint
        index.save(fd);
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

    auto header(loadHeader(fd));

    return { new Detail(std::get<1>(header), std::get<0>(header)
                        , std::move(fd), (openMode == OpenMode::readOnly)) };
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

    std::uint8_t version(0);

    if (createMode != CreateMode::append) {
        // empty file -> write header
        version = saveHeader(fd, options);
    } else {
        auto size(fileSize(fd));
        if (!size) {
            // empty -> write header
            version = saveHeader(fd, options);
        } else {
            const auto current(loadHeader(fd));
            if (std::get<0>(current) != options) {
                LOGTHROW(warn1, std::runtime_error)
                    << "Unable to append file " << path
                    << ": file has different configuration.";
            }
            version = std::get<1>(current);
        }
    }

    return { new Detail(version, options, std::move(fd), false) };
}

class Tilar::Device {
public:
    typedef char char_type;

    // write constructor
    Device(Tilar::Detail &owner, const FileIndex &index
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
    const FileIndex index;

    off_t start;
    off_t pos;
    off_t end;
};

class Tilar::Sink {
public:
    typedef char char_type;
    typedef boost::iostreams::sink_tag category;

    Sink(Tilar::Detail &owner, const FileIndex &index)
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

    Source(Tilar::Detail &owner, const FileIndex &index) {
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
    Stream(Tilar::Detail &owner, const FileIndex &index)
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
    Stream(Tilar::Detail &owner, const FileIndex &index)
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

OStream::pointer Tilar::output(const FileIndex &index)
{
    return std::make_shared<Sink::Stream>(detail(), index);
}

IStream::pointer Tilar::input(const FileIndex &index)
{
    return std::make_shared<Source::Stream>(detail(), index);
}

} } // namespace vadstena::tilestorage
