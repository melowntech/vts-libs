#include <sstream>

#include <boost/filesystem.hpp>
#include <boost/format.hpp>
#include <boost/utility/in_place_factory.hpp>

#include "utility/path.hpp"
#include "utility/magic.hpp"

#include "./tar.hpp"
#include "../io.hpp"
#include "../error.hpp"
#include "../tileop.hpp"

namespace vadstena { namespace tilestorage {

namespace fs = boost::filesystem;

namespace {

const std::string ConfigName("mapConfig.json");
const std::string TileIndexName("index.bin");

class StringIStream : public IStream {
public:
    template <typename Type>
    StringIStream(Type type, const std::string &path
                  , const utility::tar::Reader::Data &data
                  , std::time_t time)
        : IStream(type)
        , path_(path), s_(std::string(data.data(), data.size()))
        , stat_{ data.size(), time }
    {}

    virtual std::istream& get() UTILITY_OVERRIDE {
        return s_;
    }

    virtual void close() UTILITY_OVERRIDE {}

    virtual std::string name() const UTILITY_OVERRIDE {
        return path_.string();
    };

    virtual FileStat stat_impl() const UTILITY_OVERRIDE { return stat_; }

private:
    fs::path path_;
    std::istringstream s_;
    FileStat stat_;
};

template <typename Type>
IStream::pointer readFile(utility::tar::Reader &reader
                          , const TarDriver::Record &record
                          , Type type)
{
    return std::make_shared<StringIStream>
        (type, record.path, reader.readData(record.block, record.size)
         , record.time);
}

} // namespace

TarDriver::TarDriver(const boost::filesystem::path &root
         , OpenMode mode)
    : ReadOnlyDriver(mode == OpenMode::readOnly)
    , tarPath_(absolute(root)), reader_(tarPath_)
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
    }

    auto fsrc(src->find(tileId));
    if (fsrc == src->end()) {
        LOGTHROW(err1, std::runtime_error)
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
    }

    if (!r->size) {
        LOGTHROW(err1, std::runtime_error)
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
    }

    auto fsrc(src->find(tileId));
    if (fsrc == src->end()) {
        LOGTHROW(err1, std::runtime_error)
            << "No data for " << tileId << " " << desc << ".";
    }

    return { fsrc->second.size, fsrc->second.time };
}

std::string TarDriver::detectType_impl(const std::string &location)
{
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
