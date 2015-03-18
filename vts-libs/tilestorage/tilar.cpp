#include <cerrno>
#include <ctime>
#include <stdexcept>
#include <system_error>
#include <algorithm>
#include <sstream>

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

#include <boost/noncopyable.hpp>
#include <boost/crc.hpp>
#include <boost/uuid/nil_generator.hpp>

#include "utility/filedes.hpp"
#include "utility/enum.hpp"
#include "utility/raise.hpp"

#include "./tilar.hpp"
#include "./tilar-io.hpp"
#include "./error.hpp"

namespace vadstena { namespace tilestorage {

namespace fs = boost::filesystem;
using utility::Filedes;
typedef Tilar::FileIndex FileIndex;

namespace {

typedef std::uint8_t Version;

namespace header_constants {
    const std::size_t size(28);
    const std::array<std::uint8_t, 5> magic({{ 'T', 'I', 'L', 'A', 'R' }});
    const Version version(0);

    namespace index {
        const int version(5);
        const int binaryOrder(version + 1);
        const int filesPerTile(binaryOrder + 1);
        const int uuid(filesPerTile + 1);
        const int crc32(uuid + 16);
    }  // namespace index
} // namespace header

namespace index_constants {
    const std::size_t size(24);
    const std::array<std::uint8_t, 4> magic({{ 'T', 'I', 'D', 'X' }});

    namespace index {
        const int previous(4);
        const int overhead(previous + 4);
        const int timestamp(overhead + 4);
        const int crc32(timestamp + 8);
    }  // namespace index
} // namespace index_constants

template <typename T, size_t S>
void serialize(std::array<std::uint8_t, S> &out, int index, const T &value)
{
    *reinterpret_cast<T*>(out.data() + index) = value;
    std::copy(reinterpret_cast<const std::uint8_t*>(&value)
              , reinterpret_cast<const std::uint8_t*>(&value) + sizeof(T)
              , out.data() + index);
}

template <typename T, size_t S>
void deserialize(const std::array<std::uint8_t, S> &in, int index
                 , T &value)
{
    std::copy(in.data() + index, in.data() + index + sizeof(T)
              , reinterpret_cast<std::uint8_t*>(&value));
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
    case Tilar::CreateMode::appendOrTruncate:
        return O_RDWR | O_CREAT;
    }
    throw;
}

off_t seekFromEnd(const Filedes &fd, off_t pos = 0)
{
    LOG(debug)
        << "Seeking to " << pos << " bytes from the end in file "
        << fd.path() << ".";
    const auto p(::lseek(fd, -pos, SEEK_END));
    if (-1 == p) {
        std::system_error e
            (errno, std::system_category()
             , utility::formatError
             ("Failed to seek %s bytes from the end of the tilar file %s."
              , pos, fd.path()));
        LOG(err1) << e.what();
        throw e;
    }
    return p;
}

off_t seekFromStart(const Filedes &fd, off_t pos = 0)
{
    LOG(debug)
        << "Seeking to " << pos << " bytes from the start in file "
        << fd.path() << ".";
    const auto p(::lseek(fd, pos, SEEK_SET));
    if (-1 == p) {
        std::system_error e
            (errno, std::system_category()
             , utility::formatError
             ("Failed to seek %s bytes from the start of the tilar file %s."
              , pos, fd.path()));
        LOG(err1) << e.what();
        throw e;
    }
    return p;
}

off_t fileSize(const Filedes &fd)
{
    struct ::stat buf;
    if (-1 == ::fstat(fd, &buf)) {
        std::system_error e
            (errno, std::system_category()
             , utility::formatError
             ("Failed to stat tilar file %s.", fd.path()));
        LOG(err1) << e.what();
        throw e;
    }
    return buf.st_size;
}

off_t truncate(const Filedes &fd, off_t size)
{
    if (-1 == ::ftruncate(fd, size)) {
        std::system_error e
            (errno, std::system_category()
             , utility::formatError
             ("Failed to truncate tilar file %s.", fd.path()));
        LOG(err1) << e.what();
        throw e;
    }
    return seekFromStart(fd, size);
}

template <typename Block>
void write(const Filedes &fd, const Block &block)
{
    auto left(block.size() * sizeof(typename Block::value_type));
    const auto *data(reinterpret_cast<const unsigned char*>(block.data()));
    while (left) {
        auto bytes(::write(fd, data, left));
        if (-1 == bytes) {
            // if (EINTR == errno) { continue; }
            std::system_error e
                (errno, std::system_category()
                 , utility::formatError
                 ("Failed to write to tilar file %s.", fd.path()));
            LOG(err1) << e.what();
            throw e;
        }
        left -= bytes;
        data += left;
    }
}

template <typename Block>
void read(const Filedes &fd, Block &block)
{
    auto size(block.size() * sizeof(typename Block::value_type));
    auto left(size);
    auto *data(reinterpret_cast<unsigned char*>(block.data()));
    while (left) {
        auto bytes(::read(fd, data, left));
        if (-1 == bytes) {
            // if (EINTR == errno) { continue; }
            std::system_error e
                (errno, std::system_category()
                 , utility::formatError
                 ("Failed to read from tilar file %s.", fd.path()));
            LOG(err1) << e.what();
            throw e;
        }
        if (!bytes) {
            // EOF!
            LOGTHROW(err1, std::runtime_error)
                << "EOF while trying to read " << size
                << " bytes from file " << fd.path() << " (only "
                << (size - left) << " bytes have been read).";
        }
        left -= bytes;
        data += left;
    }
}

std::uint32_t crc(std::uint8_t version, const Tilar::Options &options)
{
    boost::crc_32_type crc;
    crc.process_byte(version);
    crc.process_byte(options.binaryOrder);
    crc.process_byte(options.filesPerTile);
    crc.process_bytes(options.uuid.data, options.uuid.size());
    return crc.checksum();
}

std::tuple<Tilar::Options, std::uint8_t> loadHeader(const Filedes &fd)
{
    LOG(info1) << "Loading archive header.";

    std::array<std::uint8_t, header_constants::size> header;
    read(fd, header);

    if (!std::equal(header_constants::magic.begin()
                    , header_constants::magic.end()
                    , header.begin()))
    {
        LOGTHROW(err1, InvalidSignature)
            << "Invalid magic in header of tilar file "
            << fd.path() << ".";
    }

    auto version(header[header_constants::index::version]);
    if (version != header_constants::version) {
        LOGTHROW(err1, std::runtime_error)
            << "Invalid version in header of tilar file "
            << fd.path() << ": " << int(version) << ".";
    }

    Tilar::Options options(header[header_constants::index::binaryOrder]
                           , header[header_constants::index::filesPerTile]);

    std::copy(header.begin() + header_constants::index::uuid
              , header.begin()
              + (header_constants::index::uuid + options.uuid.size())
              , options.uuid.data);

    std::uint32_t savedCrc;
    deserialize(header, header_constants::index::crc32, savedCrc);
    auto computedCrc(crc(version, options));
    if (computedCrc != savedCrc) {
        LOGTHROW(err1, InvalidSignature)
            << "Invalid CRC32 of archive file " << fd.path() << "; expected 0x"
            << std::hex << computedCrc << ", encoutered 0x"
            << savedCrc << ".";
    }

    return std::tuple<Tilar::Options, std::uint8_t>(options, version);
}

Version saveHeader(const Filedes &fd, const Tilar::Options &options)
{
    LOG(info1) << "Saving archive header.";

    auto version(header_constants::version);

    std::array<std::uint8_t, header_constants::size> header;
    std::copy(header_constants::magic.begin(), header_constants::magic.end()
              , header.begin());
    header[header_constants::index::version] = version;

    header[header_constants::index::binaryOrder] = options.binaryOrder;
    header[header_constants::index::filesPerTile] = options.filesPerTile;

    std::copy(options.uuid.data, options.uuid.data + options.uuid.size()
              , header.begin() + header_constants::index::uuid);
    serialize(header, header_constants::index::crc32, crc(version, options));

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

class ArchiveIndex : boost::noncopyable {
public:
    struct Slot {
        std::uint32_t start;
        std::uint32_t size;

        Slot() : start(0), size(0) {}

        bool valid() const { return start > 0; }

        std::uint32_t end() const { return start + size; }
    };

    ArchiveIndex(const Tilar::Options &options)
        : options_(options)
        , edge_(edge(options))
        , rowSkip_(edge_)
        , typeSkip_(tiles(options))
        , grid_(files(options), Slot())
        , previous_(0), overhead_(0), timestamp_(0)
        , changed_(false), loadedFrom_(0)
    {}

    /** Loads index from given file descriptor at pos bytes from the end of a
     *  file.
     */
    void load(const Filedes &fd, off_t pos, bool checkCrc = false);

    /** Saves new index to the end of the file;
     */
    void save(const Filedes &fd);

    /** Clears every slot. Index is freshened.
     */
    void clear();

    void set(const FileIndex &index, off_t start, off_t end);

    void unset(const FileIndex &index);

    const Slot& get(const FileIndex &index) const { return slot(index); }

    std::uint32_t crc(std::uint32_t overhead) const;
    bool changed() const { return changed_; }
    void freshen() { changed_ = false; }

    int savedSize() const {
        return index_constants::size + (grid_.size() * sizeof(Slot));
    }

    Tilar::Entry::list list() const;

    Tilar::Info info() const;

    void check(const FileIndex &fileIndex) const {
        if (!((fileIndex.col < edge_)
              && (fileIndex.row < edge_)
              && (fileIndex.type < options_.filesPerTile)))
        {
            LOGTHROW(err2, std::runtime_error)
                << "Index [" << fileIndex.col << ',' << fileIndex.row
                << ',' << fileIndex.type << "] out of bounds.";
        }
    }

    FileStat stat(const FileIndex &index) const;

private:
    inline Slot& slot(const FileIndex &index) {
        check(index);
        return grid_[index.col
                     + (rowSkip_ * index.row)
                     + (typeSkip_ * index.type)];
    }

    inline const Slot& slot(const FileIndex &index) const {
        check(index);
        return grid_[index.col
                     + (rowSkip_ * index.row)
                     + (typeSkip_ * index.type)];
    }

    typedef std::vector<Slot> Grid;

    const Tilar::Options options_;
    const unsigned int edge_;
    const unsigned int rowSkip_;
    const unsigned int typeSkip_;

    Grid grid_;
    std::uint32_t previous_;
    std::uint32_t overhead_;
    std::uint64_t timestamp_;

    bool changed_;
    std::uint32_t loadedFrom_;
};

std::uint32_t ArchiveIndex::crc(std::uint32_t overhead) const
{
    boost::crc_32_type crc;
    crc.process_bytes(&overhead, sizeof(overhead));
    crc.process_bytes(&timestamp_, sizeof(timestamp_));
    crc.process_bytes(grid_.data(), grid_.size());
    return crc.checksum();
}

void ArchiveIndex::save(const Filedes &fd)
{
    LOG(info1) << "Saving new archive index to file.";
    auto offset(fileSize(fd));

    // save header first
    std::array<std::uint8_t, index_constants::size> header;
    std::copy(index_constants::magic.begin(), index_constants::magic.end()
              , header.begin());

    auto overhead(overhead_);
    // take into account index size of index if there is already some index
    // present in the file
    if (loadedFrom_) { overhead += savedSize(); }

    // update timestamp
    timestamp_ = std::time(nullptr);

    serialize(header, index_constants::index::previous, loadedFrom_);
    serialize(header, index_constants::index::overhead, overhead);
    serialize(header, index_constants::index::timestamp, timestamp_);
    serialize(header, index_constants::index::crc32, crc(overhead));

    seekFromEnd(fd);
    write(fd, header);
    write(fd, grid_);

    // update overhead
    overhead_ = overhead;
    previous_ = loadedFrom_;
    loadedFrom_ = offset;
}

void ArchiveIndex::load(const Filedes &fd, off_t pos, bool checkCrc)
{
    LOG(info1) << "Loading archive index from file.";
    auto start(seekFromEnd(fd, pos));

    // load header first
    std::array<std::uint8_t, index_constants::size> header;
    read(fd, header);

    if (!std::equal(index_constants::magic.begin()
                    , index_constants::magic.end()
                    , header.begin()))
    {
        LOGTHROW(err1, InvalidSignature)
            << "Invalid magic in index of tilar file "
            << fd.path() << " at position "
            << pos << ".";
    }

    std::uint32_t savedCrc;
    deserialize(header, index_constants::index::previous, previous_);
    deserialize(header, index_constants::index::overhead, overhead_);
    deserialize(header, index_constants::index::timestamp, timestamp_);
    deserialize(header, index_constants::index::crc32, savedCrc);

    read(fd, grid_);

    if (checkCrc) {
        auto computedCrc(crc(overhead_));
        if (computedCrc != savedCrc) {
            LOGTHROW(err1, InvalidSignature)
                << "Invalid CRC32 of archive index in file " << fd.path()
                << " at position " << pos << "; expected 0x"
                << std::hex << computedCrc << ", encoutered 0x"
                << savedCrc << ".";
        }
    }

    loadedFrom_ = start;
    changed_ = false;
}

void ArchiveIndex::clear()
{
    grid_.assign(grid_.size(), Slot());
    changed_ = false;
}

void ArchiveIndex::set(const FileIndex &index, off_t start, off_t end)
{
    const auto size(end - start);
    auto &s(slot(index));

    if (s.valid()) {
        overhead_ += size;
    }

    changed_ = (s.start != start);
    s.start = start;
    s.size = size;
}

void ArchiveIndex::unset(const FileIndex &index)
{
    auto &s(slot(index));

    if (s.valid()) {
        overhead_ += s.size;
        s.start = s.size = 0;
        changed_ = true;
    }
}

Tilar::Entry::list ArchiveIndex::list() const
{
    Tilar::Entry::list list;

    auto fgrid(grid_.begin());
    FileIndex index;
    for (index.type = 0; index.type < options_.filesPerTile; ++index.type) {
        for (index.row = 0; index.row < edge_; ++index.row) {
            for (index.col = 0; index.col < edge_; ++index.col, ++fgrid) {
                if (!fgrid->valid()) { continue; }
                list.emplace_back(index, fgrid->start, fgrid->size);
            }
        }
    }

    return list;
}

Tilar::Info ArchiveIndex::info() const
{
    return { loadedFrom_, previous_, overhead_, timestamp_ };
}

FileStat ArchiveIndex::stat(const FileIndex &index) const
{
    // what time do uncommitted files have?
    return { std::size_t(slot(index).size), std::time_t(timestamp_)
            , "application/octet-stream" };
}

} // namespace

struct Tilar::Detail
{
    typedef std::shared_ptr<Detail> pointer;

    Detail(const Detail&) = delete;
    Detail& operator=(const Detail&) = delete;
    Detail(Detail&&) = delete;
    Detail& operator=(Detail&&) = delete;

    Detail(std::uint8_t version, const Options &options
           , Filedes &&srcFd, bool readOnly
           , std::uint32_t indexOffset)
        : version(version), options(options), fd(std::move(srcFd))
        , readOnly(readOnly), index(options)
        , checkpoint(fileSize(fd)), tx(0)
        , ignoreInterrupts(false)
        , indexOffset(indexOffset)
    {
        loadIndex();
    }

    ~Detail();

    void loadIndex() {
        if (indexOffset > header_constants::size) {
            index.load(fd, checkpoint - indexOffset, true);
        } else if (checkpoint > off_t(header_constants::size)) {
            // more bytes than header
            if (checkpoint < (index.savedSize() + header_constants::size)) {
                LOGTHROW(err1, InvalidSignature)
                    << "File " << fd.path() << " is too short.";
            }
            // load index but do not chech CRC
            index.load(fd, index.savedSize(), true);
        } else {
            index.clear();
        }
    }

    void commitChanges();

    void discardChanges();

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
        this->index.check(index);

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

    bool changed() {
        return (!readOnly && (index.changed() || fileSize(fd) != checkpoint));
    }

    FileStat stat(const FileIndex &fileIndex) const {
        return index.stat(fileIndex);
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

    bool ignoreInterrupts;
    std::uint32_t indexOffset;
};

void Tilar::Detail::commitChanges()
{
    if (tx) {
        LOGTHROW(err2, PendingTransaction)
            << "Pending transaction in archive " << fd.path() << ".";
    }

    if (changed()) {
        // save index and remember new checkpoint
        index.save(fd);
        checkpoint = fileSize(fd);
        index.freshen();
    }
}

void Tilar::Detail::discardChanges()
{
    if (tx) {
        LOGTHROW(err2, PendingTransaction)
            << "Pending transaction in archive " << fd.path() << ".";
    }

    if (changed()) {
        truncate(fd, checkpoint);
        loadIndex();
    }
}

Tilar::Detail::~Detail()
{
    if (changed()) {
        // unflushed -> rollback
        LOG(warn2)
            << "File " << fd.path() << " was not flushed, discarding changes.";
        tx = 0;
        discardChanges();
    }
}

Tilar::Tilar(const Tilar::Detail::pointer &detail)
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
    Filedes fd(::open(path.string().c_str(), flags(openMode)), path);
    if (-1 == fd) {
        std::system_error e
            (errno, std::system_category()
             , utility::formatError
             ("Failed to open tilar file %s.", path));
        LOG(err1) << e.what();
        throw e;
    }

    auto header(loadHeader(fd));

    return { std::make_shared<Detail>
            (std::get<1>(header), std::get<0>(header)
             , std::move(fd), (openMode == OpenMode::readOnly), 0) };
}

Tilar Tilar::open(const fs::path &path, std::uint32_t indexOffset)
{
    Filedes fd(::open(path.string().c_str(), flags(OpenMode::readOnly)), path);
    if (-1 == fd) {
        std::system_error e
            (errno, std::system_category()
             , utility::formatError
             ("Failed to open tilar file %s.", path));
        LOG(err1) << e.what();
        throw e;
    }

    auto header(loadHeader(fd));

    return { std::make_shared<Detail>
            (std::get<1>(header), std::get<0>(header)
             , std::move(fd), true, indexOffset) };
}

Tilar Tilar::create(const fs::path &path, const Options &options
                    , CreateMode createMode)
{
    LOG(info1)
        << "Creating tilar file at " << path << " in "
        << createMode << " mode.";

    Filedes fd
        (::open(path.string().c_str(), flags(createMode)
                , S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP | S_IROTH | S_IWOTH)
         , path);
    if (-1 == fd) {
        std::system_error e
            (errno, std::system_category()
             , utility::formatError
             ("Failed to create tilar file %s.", path));
        throw e;
    }

    if (utility::in
        (createMode, CreateMode::failIfExists, CreateMode::truncate))
    {
        // empty file -> write header
        return { std::make_shared<Detail>
                 (saveHeader(fd, options), options, std::move(fd), false, 0) };
    }

    auto size(fileSize(fd));
    if (!size) {
        // empty -> write header
        return { std::make_shared<Detail>
                (saveHeader(fd, options), options, std::move(fd), false, 0) };
    } else if (createMode == CreateMode::append) {
        const auto current(loadHeader(fd));
        if (std::get<0>(current) != options) {
            // different setup
            LOGTHROW(err1, std::runtime_error)
                << "Unable to append file " << path
                << ": file has different configuration.";
        }
        return { std::make_shared<Detail>
                (std::get<1>(current), options, std::move(fd), false, 0) };
    }

    boost::optional<std::uint8_t> version;
    try {
        const auto current(loadHeader(fd));
        if (std::get<0>(current) != options) {
            LOG(warn1)
                << "File " << path << " has different configuration"
                << ", truncating.";
        } else {
            version = std::get<1>(current);
        }
    } catch (const std::exception &e) {
        LOG(warn1)
            << "File " << path << " has invalid header (" << e.what()
            << "): truncating";
    }

    if (!version) {
        truncate(fd, 0);
        version = saveHeader(fd, options);
    }

    return { std::make_shared<Detail>
             (*version, options, std::move(fd), false, 0) };
}

class Tilar::Device {
public:
    typedef char char_type;

    // write constructor
    Device(const Tilar::Detail::pointer &owner, const FileIndex &index
           , off_t start)
        : owner(owner), fd(owner->fd), index(index)
        , start(start), pos(start), end(0)
    {
        owner->begin(index, start);
    }

    // read constructor
    Device(const Tilar::Detail::pointer &owner, const FileIndex &index)
        : owner(owner), fd(owner->fd), index(index)
        , start(0), pos(0), end(0)
    {
        const auto &slot(owner->index.get(index));
        if (!slot.valid()) {
            LOGTHROW(err1, std::runtime_error)
                << "File [" << index.col << ',' << index.row
                << ',' << index.type << " does not exist in the archive "
                << fd.path() << ".";
        }

        // update
        pos = start = slot.start;
        end = slot.end();
    }

    ~Device() {
        if (end || (pos == start)) { return; }
        try {
            if (!std::uncaught_exception()) {
                LOG(warn3) << "File write was not finished!";
            }
            LOG(warn1) << "Rolling back file.";
            owner->rollback();
        } catch (...) {}
    }

    void commit() {
        if (start != pos) {
            // commit transaction
            owner->commit(pos);
            // pretend there was nothing written
            start = pos;
        }
    }

    std::string name() const {
        std::ostringstream os;
        os << fd.path().string()
           << ':' << index.row
           << ',' << index.col
           << ',' << index.type;
        return os.str();
    }

    void rewind(off_t newPos) { pos = start + newPos; }

    bool ignoreInterrupts() const { return owner->ignoreInterrupts; }

    FileStat stat() const { return owner->stat(index); }

    Tilar::Detail::pointer owner;
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

    Sink(const Tilar::Detail::pointer &owner, const FileIndex &index)
        : device_(std::make_shared<Device>
                  (owner, index, seekFromEnd(owner->fd)))
    {}

    void commit() { device_->commit(); }
    std::string name() const { return device_->name(); }

    std::streamsize write(const char *s, std::streamsize n);

    FileStat stat() const { return device_->stat(); }

    class Stream;

private:
    std::shared_ptr<Device> device_;
};

class Tilar::Source {
public:
    typedef char char_type;
    struct category : boost::iostreams::device_tag
                    , boost::iostreams::input_seekable {};

    Source(const Tilar::Detail::pointer &owner, const FileIndex &index)
        : device_(std::make_shared<Device>(owner, index))
    {}

    std::string name() const { return device_->name(); }

    std::streamsize read(char *data, std::streamsize size
                         , boost::iostreams::stream_offset pos)
    {
        // read data from given position
        auto bytes(read_impl(data, size, pos + device_->start));
        // update position after read block
        device_->rewind(pos + bytes);
        return bytes;
    }

    std::streamsize read(char *data, std::streamsize size);

    std::streampos seek(boost::iostreams::stream_offset off
                        , std::ios_base::seekdir way);

    FileStat stat() const { return device_->stat(); }

    ReadOnlyFd readOnlyfd() {
        return { device_->fd.get(), device_->start, device_->end };
    }

    class Stream;

private:
    std::streamsize read_impl(char *data, std::streamsize size
                              , boost::iostreams::stream_offset pos);

    std::shared_ptr<Device> device_;
};

class Tilar::Sink::Stream
    : private boost::iostreams::stream_buffer<Tilar::Sink>
    , public tilestorage::OStream
{
public:
    Stream(const Tilar::Detail::pointer &owner, const FileIndex &index)
        : buffer_(owner, index), stream_(&buffer_) {}

    virtual std::ostream& get() { return stream_; }
    virtual void close() {
        stream_.flush();
        buffer_->commit();
        buffer_.close();
    }
    virtual std::string name() const {
        // stream_buffer has only non-const version of operator-> :(
        return const_cast<decltype(buffer_)&>(buffer_)->name();
    }

    virtual FileStat stat() const {
        // stream_buffer has only non-const version of operator-> :(
        return const_cast<decltype(buffer_)&>(buffer_)->stat();
    }

private:
    boost::iostreams::stream_buffer<Tilar::Sink> buffer_;
    std::ostream stream_;
};

class Tilar::Source::Stream
    : private boost::iostreams::stream_buffer<Tilar::Source>
    , public tilestorage::IStream
{
public:
    Stream(const Tilar::Detail::pointer &owner, const FileIndex &index)
        : buffer_(owner, index), stream_(&buffer_) {}

    virtual std::istream& get() { return stream_; }
    virtual void close() { buffer_.close(); }
    virtual std::string name() const {
        // stream_buffer has only non-const version of operator-> :(
        return const_cast<decltype(buffer_)&>(buffer_)->name();
    }

    virtual std::size_t read(char *buf, std::size_t size
                             , std::istream::pos_type off)
    {
        return buffer_->read(buf, size, off);
    }

    virtual FileStat stat() const {
        // stream_buffer has only non-const version of operator-> :(
        return const_cast<decltype(buffer_)&>(buffer_)->stat();
    }

    virtual boost::optional<ReadOnlyFd> fd() {
        return buffer_->readOnlyfd();
    }

private:
    boost::iostreams::stream_buffer<Tilar::Source> buffer_;
    std::istream stream_;
};

std::streamsize Tilar::Sink::write(const char *data, std::streamsize size)
{
    auto &pos(device_->pos);
    const auto &fd(device_->fd);
    for (;;) {
        auto bytes(::pwrite(fd, data, size, pos));
        if (-1 == bytes) {
            if ((EINTR == errno) && device_->ignoreInterrupts()) {
                continue;
            }
            std::system_error e
                (errno, std::system_category()
                 , utility::formatError
                 ("Unable to write to tilar file %s.", fd.path()));
            throw e;
        }
        pos += bytes;
        return bytes;
    }
}

std::streamsize
Tilar::Source::read_impl(char *data, std::streamsize size
                         , boost::iostreams::stream_offset pos)
{
    // trim if out of range
    auto end(device_->end);
    if (size > (end - pos)) { size = end - pos; }

    if (!size) { return size; }

    const auto &fd(device_->fd);
    for (;;) {
        auto bytes(::pread(fd, data, size, pos));
        if (-1 == bytes) {
            if ((EINTR == errno) && device_->ignoreInterrupts()) {
                continue;
            }
            std::system_error e
                (errno, std::system_category()
                 , utility::formatError
                 ("Unable to read from tilar file %s.", fd.path()));
            throw e;
        }
        return bytes;
    }
}

std::streamsize Tilar::Source::read(char *data, std::streamsize size)
{
    auto bytes(read_impl(data, size, device_->pos));
    device_->pos += bytes;
    return bytes;
}

std::streampos Tilar::Source::seek(boost::iostreams::stream_offset off
                                   , std::ios_base::seekdir way)
{
    std::int64_t newPos(0);

    switch (way) {
    case std::ios_base::beg:
        newPos = device_->start + off;
        break;

    case std::ios_base::end:
        newPos = device_->end + off;
        break;

    case std::ios_base::cur:
        newPos = device_->pos + off;
        break;

    default: // shut up compiler!
        break;
    };

    if (newPos < std::int64_t(device_->start)) {
        device_->pos = device_->start;
    } else if (newPos > std::int64_t(device_->end)) {
        device_->pos = device_->end;
    } else {
        device_->pos = newPos;
    }

    return (device_->pos - device_->start);
}

void Tilar::commit()
{
    detail().commitChanges();
}

void Tilar::rollback()
{
    detail().discardChanges();
}

OStream::pointer Tilar::output(const FileIndex &index)
{
    return std::make_shared<Sink::Stream>(detail_, index);
}

IStream::pointer Tilar::input(const FileIndex &index)
{
    return std::make_shared<Source::Stream>(detail_, index);
}

std::size_t Tilar::size(const FileIndex &index)
{
    return detail().index.get(index).size;
}

FileStat Tilar::stat(const FileIndex &index)
{
    return detail().index.stat(index);
}

void Tilar::remove(const FileIndex &index)
{
    detail().index.unset(index);
}

Tilar::Entry::list Tilar::list() const
{
    return detail().index.list();
}

Tilar::Info Tilar::info() const
{
    return detail().index.info();
}

const Tilar::Options& Tilar::options() const
{
    return detail().options;
}

bool Tilar::ignoreInterrupts() const
{
    return detail().ignoreInterrupts;
}

void Tilar::ignoreInterrupts(bool value)
{
    detail().ignoreInterrupts = value;
}

void Tilar::expect(const Options &options)
{
    if (options != detail().options) {
        LOGTHROW(err1, std::runtime_error)
            << "Expectation failed: file " << detail().fd.path()
            << " has different configuration.";
    }
}

Tilar::Options::Options(unsigned int binaryOrder, unsigned int filesPerTile)
    : binaryOrder(binaryOrder), filesPerTile(filesPerTile)
    , uuid(boost::uuids::nil_uuid())
{}

bool Tilar::Options::operator==(const Options &o) const
{
    return ((binaryOrder == o.binaryOrder)
            && (filesPerTile == o.filesPerTile)
            && (uuid == o.uuid));
}

} } // namespace vadstena::tilestorage
