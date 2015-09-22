#include <system_error>

#include <unistd.h>

#include <boost/filesystem.hpp>
#include <boost/format.hpp>
#include <boost/utility/in_place_factory.hpp>

#include "utility/path.hpp"
#include "utility/magic.hpp"
#include "utility/raise.hpp"

#include "./tar.hpp"
#include "../io.hpp"
#include "../../storage/error.hpp"
#include "../tileop.hpp"

namespace vadstena { namespace tilestorage {

namespace fs = boost::filesystem;

namespace {

const std::string ConfigName("mapConfig.json");
const std::string TileIndexName("index.bin");

std::streamsize IOBufferSize(1 << 16);

struct TarDevice {
    typedef char char_type;
    struct category : boost::iostreams::device_tag
                    , boost::iostreams::input_seekable {};

    TarDevice(const std::string &path
              , const utility::tar::Reader::Filedes &fd)
        : path_(path), fd_(fd.fd, fd.start, fd.end, true)
        , pos_(fd.start)
    {}

    std::streampos seek(boost::iostreams::stream_offset off
                        , std::ios_base::seekdir way);

    std::streamsize read(char *data, std::streamsize size
                         , boost::iostreams::stream_offset pos)
    {
        // read data from given position
        auto bytes(read_impl(data, size, pos + fd_.start));
        // update position after read block
        pos_ = fd_.start + pos + bytes;
        return bytes;
    }

    std::streamsize read(char *data, std::streamsize size) {
        auto bytes(read_impl(data, size, pos_));
        pos_ += bytes;
        return bytes;
    }

    storage::ReadOnlyFd fd() { return fd_; }

private:
    std::streamsize read_impl(char *data, std::streamsize size
                              , boost::iostreams::stream_offset pos);

    fs::path path_;
    storage::ReadOnlyFd fd_;
    boost::iostreams::stream_offset pos_;
};

class TarIStream : public IStream {
public:
    template <typename Type>
    TarIStream(Type type, const std::string &path
               , const utility::tar::Reader::Filedes &fd
               , std::time_t time)
        : IStream(type)
        , path_(path)
        , stat_{ fd.end - fd.start, time }
        , buffer_(path, fd), stream_(&buffer_)
    {
        stream_.exceptions(std::ios::badbit | std::ios::failbit);
        buf_.reset(new char[IOBufferSize]);
        buffer_.pubsetbuf(buf_.get(), IOBufferSize);
    }

    virtual std::istream& get() UTILITY_OVERRIDE {
        return stream_;
    }

    virtual void close() UTILITY_OVERRIDE {}

    virtual std::string name() const UTILITY_OVERRIDE {
        return path_.string();
    };

    virtual FileStat stat_impl() const UTILITY_OVERRIDE { return stat_; }

    virtual boost::optional<storage::ReadOnlyFd> fd() UTILITY_OVERRIDE
    {
        return buffer_->fd();
    }

    virtual std::size_t read(char *buf, std::size_t size
                             , std::istream::pos_type off)
        UTILITY_OVERRIDE
    {
        return buffer_->read(buf, size, off);
    }

private:
    fs::path path_;
    FileStat stat_;

    std::unique_ptr<char[]> buf_;
    boost::iostreams::stream_buffer<TarDevice> buffer_;
    std::istream stream_;
};

std::streampos TarDevice::seek(boost::iostreams::stream_offset off
                               , std::ios_base::seekdir way)
{
    std::int64_t newPos(0);

    switch (way) {
    case std::ios_base::beg:
        newPos = fd_.start + off;
        break;

    case std::ios_base::end:
        newPos = fd_.end + off;
        break;

    case std::ios_base::cur:
        newPos = pos_ + off;
        break;

    default: // shut up compiler!
        break;
    };

    if (newPos < std::int64_t(fd_.start)) {
        pos_ = fd_.start;
    } else if (newPos > std::int64_t(fd_.end)) {
        pos_ = fd_.end;
    } else {
        pos_ = newPos;
    }

    return (pos_ - fd_.start);
}

std::streamsize TarDevice::read_impl(char *data, std::streamsize size
                                     , boost::iostreams::stream_offset pos)
{
    // trim if out of range
    auto end(fd_.end);
    if (size > std::streamsize(end - pos)) {
        size = end - pos;
    }

    if (!size) { return size; }

    auto bytes(::pread(fd_.fd, data, size, pos));
    if (-1 == bytes) {
        std::system_error e
            (errno, std::system_category()
             , utility::formatError
             ("Unable to read from tilar file %s.", path_));
        throw e;
    }
    return bytes;
}

template <typename Type>
IStream::pointer readFile(utility::tar::Reader &reader
                          , const TarDriver::Record &record
                          , Type type)
{
    return std::make_shared<TarIStream>
        (type, record.path, reader.filedes(record.block, record.size)
         , record.time);
}

} // namespace

TarDriver::TarDriver(const boost::filesystem::path &root
         , OpenMode mode, const DetectionContext&)
    : ReadOnlyDriver(mode == OpenMode::readOnly)
    , tarPath_(absolute(root)), reader_(tarPath_)
    , openStat_(FileStat::stat(tarPath_))
{
    utility::tar::Block block;
    TileId tileId;
    TileFile type;
    const auto &header(block.header);
    while (reader_.read(block)) {
        if (!header.valid()) {
            continue;
        }

        if (header.isFile()) {
            const auto f(header.getPath().filename());
            Record record(f.string(), reader_.cursor(), header.getSize()
                          , header.getTime());

            const auto filename(f.string());
            if (filename == ConfigName) {
                configFile_ = record;
            } else if (filename == TileIndexName) {
                indexFile_ = record;
            } else {
                if (fromFilename(tileId, type, f.string())) {
                    switch (type) {
                    case TileFile::meta:
                        metaMap_.insert(FileMap::value_type(tileId, record));
                        break;

                    case TileFile::mesh:
                        meshMap_.insert(FileMap::value_type(tileId, record));
                        break;

                    case TileFile::atlas:
                        atlasMap_.insert(FileMap::value_type(tileId, record));
                        break;

                    default:
                        throw "Unexpected TileFile value. "
                            "Go fix your program.";
                    }
                }
            }
        }

        // skip file/whatever content
        reader_.skip(header);
    }
}

TarDriver::~TarDriver() {}

IStream::pointer TarDriver::input_impl(File type) const
{
    const Record *r{};
    const char *desc{};

    switch (type) {
    case File::config:
        desc = "config";
        r = &configFile_;
        break;

    case File::tileIndex:
        desc = "tile index";
        r = &indexFile_;
        break;

    default: break;
    }

    if (!r->size) {
        LOGTHROW(err1, std::runtime_error)
            << "No data for " << desc << ".";
    }

    return readFile(reader_, *r, type);
}

IStream::pointer TarDriver::input_impl(const TileId tileId, TileFile type)
    const
{
    const FileMap *src{};
    const char *desc{};

    switch (type) {
    case TileFile::meta:
        src = &metaMap_;
        desc = "metatile";
        break;

    case TileFile::mesh:
        src = &meshMap_;
        desc = "mesh";
        break;

    case TileFile::atlas:
        src = &atlasMap_;
        desc = "atlas";
        break;

    default: throw "Unexpected TileFile value. Go fix your program.";
    }

    auto fsrc(src->find(tileId));
    if (fsrc == src->end()) {
        LOGTHROW(err1, storage::NoSuchFile)
            << "No data for " << tileId << " " << desc << ".";
    }
    return readFile(reader_, fsrc->second, type);
}

FileStat TarDriver::stat_impl(File type) const
{
    const Record *r{};
    const char *desc{};

    switch (type) {
    case File::config:
        desc = "config";
        r = &configFile_;
        break;

    case File::tileIndex:
        desc = "tile index";
        r = &indexFile_;
        break;

    default: break;
    }

    if (!r->size) {
        LOGTHROW(err1, storage::NoSuchFile)
            << "No data for " << desc << ".";
    }

    return { r->size, r->time };
}

FileStat TarDriver::stat_impl(const TileId tileId, TileFile type) const
{
    const FileMap *src{};
    const char *desc{};

    switch (type) {
    case TileFile::meta:
        src = &metaMap_;
        desc = "metatile";
        break;

    case TileFile::mesh:
        src = &meshMap_;
        desc = "mesh";
        break;

    case TileFile::atlas:
        src = &atlasMap_;
        desc = "atlas";
        break;

    default: throw "Unexpected TileFile value. Go fix your program.";
    }

    auto fsrc(src->find(tileId));
    if (fsrc == src->end()) {
        LOGTHROW(err1, storage::NoSuchFile)
            << "No data for " << tileId << " " << desc << ".";
    }

    return { fsrc->second.size, fsrc->second.time };
}

bool TarDriver::externallyChanged_impl() const
{
    return openStat_.changed(FileStat::stat(tarPath_));
}

std::string TarDriver::detectType_impl(DetectionContext &context
                                       , const std::string &location)
{
    // TODO: record mime type in the context and use cached value if found
    (void) context;
    try {
        if (utility::Magic().mime(location) == "application/x-tar") {
            return "tar";
        }
    } catch (const std::exception&) {}
    return {};
}

const std::string TarDriver::help
("Read-only storage driver for accessing tiles inside an (ustar) "
 "tar archive.");

} } // namespace vadstena::tilestorage
