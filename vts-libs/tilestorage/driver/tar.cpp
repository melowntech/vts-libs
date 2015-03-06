#include <sstream>

#include <boost/filesystem.hpp>
#include <boost/format.hpp>
#include <boost/utility/in_place_factory.hpp>

#include "utility/path.hpp"
#include "utility/magic.hpp"

#include "./tar.hpp"
#include "../io.hpp"
#include "../error.hpp"

namespace vadstena { namespace tilestorage {

namespace fs = boost::filesystem;

namespace {

const std::string ConfigName("mapConfig.json");
const std::string TileIndexName("index.bin");

class StringIStream : public IStream {
public:
    StringIStream(const std::string &path
                  , const utility::tar::Reader::Data &data)
        : path_(path), s_(std::string(data.data(), data.size()))
    {}

    virtual ~StringIStream() {}

    virtual std::istream& get() UTILITY_OVERRIDE {
        return s_;
    }

    virtual void close() UTILITY_OVERRIDE {}

    virtual std::string name() const UTILITY_OVERRIDE {
        return path_.string();
    };

private:
    fs::path path_;
    std::istringstream s_;
};

bool decodeTileId(TileId &tileId, const std::string &str)
{
    const char *p(str.c_str());
    char *e(nullptr);

    tileId.lod = std::strtoul(p, &e, 10);
    if (*e != '-') { return false; }
    p = e + 1;

    tileId.easting = std::strtoul(p, &e, 10);
    if (*e != '-') { return false; }
    p = e + 1;

    tileId.northing = std::strtoul(p, &e, 10);
    return !*e;
}

IStream::pointer readFile(utility::tar::Reader &reader
                          , const TarDriver::Record &record)
{
    return std::make_shared<StringIStream>
        (record.path, reader.readData(record.block, record.size));
}

} // namespace

TarDriver::TarDriver(const boost::filesystem::path &root
         , OpenMode mode)
    : ReadOnlyDriver(mode == OpenMode::readOnly)
    , tarPath_(root), reader_(root)
{
    utility::tar::Block block;
    TileId tileId;
    const auto &header(block.header);
    while (reader_.read(block)) {
        if (!header.valid()) {
            continue;
        }

        if (header.isFile()) {
            const auto f(header.getPath().filename());
            Record record(f.string(), reader_.cursor(), header.getSize());

            const auto filename(f.string());
            if (filename == ConfigName) {
                configFile_ = record;
            } else if (filename == TileIndexName) {
                indexFile_ = record;
            } else {
                const auto stem(f.stem().string());
                const auto ext(f.extension().string());
                FileMap *dst{};

                if (ext == ".bin") {
                    dst = &meshMap_;
                } else if (ext == ".jpg") {
                    dst = &atlasMap_;
                } else if (ext == ".meta") {
                    dst = &metaMap_;
                }

                if (dst && decodeTileId(tileId, stem)) {
                    dst->insert(FileMap::value_type(tileId, record));
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

    return readFile(reader_, *r);
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
    return readFile(reader_, fsrc->second);
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
