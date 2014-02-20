#include <fstream>

#include <boost/filesystem.hpp>
#include <boost/format.hpp>
#include <boost/utility/in_place_factory.hpp>

#include "utility/streams.hpp"

#include "./flat.hpp"
#include "../json.hpp"
#include "../io.hpp"
#include "../error.hpp"

#include "tilestorage/browser/index.html.hpp"
#include "tilestorage/browser/skydome.jpg.hpp"

namespace vadstena { namespace tilestorage {

namespace fs = boost::filesystem;

namespace {

    const std::string ConfigName("mapConfig.json");
    const std::string TileIndexName("index.bin");
    const std::string MetaIndexName("metaindex.bin");
    const std::string TransactionRoot("tx");

    const std::string filePath(Driver::File type)
    {
        switch (type) {
        case Driver::File::config: return ConfigName;
        case Driver::File::tileIndex: return TileIndexName;
        case Driver::File::metaIndex: return MetaIndexName;
        }
        throw "unknown file type";
    }

    const char *extension(Driver::TileFile type)
    {
        switch (type) {
        case Driver::TileFile::meta: return "meta";
        case Driver::TileFile::mesh: return "bin";
        case Driver::TileFile::atlas: return "jpg";
        }
        throw "unknown tile file type";
    }

    fs::path filePath(const TileId &tileId, Driver::TileFile type)
    {
        return str(boost::format("%s-%07d-%07d.%s")
                   % tileId.lod % tileId.easting % tileId.northing
                   % extension(type));
    }

    class FileOStream : public Driver::OStream {
    public:
        FileOStream(const fs::path &path)
            : f_()
        {
            f_.exceptions(std::ios::badbit | std::ios::failbit);
            f_.open(path.string(), std::ios_base::out | std::ios_base::trunc);
        }

        virtual ~FileOStream() {
            if (!std::uncaught_exception() && f_.is_open()) {
                LOG(warn3) << "File was not closed!";
            }
        }

        virtual std::ostream& get() override {
            return f_;
        }

        virtual void close() override {
            f_.close();
        }

    private:
        utility::ofstreambuf f_;
    };

    class FileIStream : public Driver::IStream {
    public:
        FileIStream(const fs::path &path)
            : f_()
        {
            f_.exceptions(std::ios::badbit | std::ios::failbit);
            f_.open(path.string());
        }

        virtual ~FileIStream() {
            if (!std::uncaught_exception() && f_.is_open()) {
                LOG(warn3) << "File was not closed!";
            }
        }

        virtual std::istream& get() override {
            return f_;
        }

        virtual void close() override {
            f_.close();
        }

    private:
        utility::ifstreambuf f_;
    };

} // namespace

FlatDriver::FlatDriver(const boost::filesystem::path &root
                       , const CreateProperties &properties
                       , CreateMode mode)
    : Driver(false)
    , root_(root), tmp_(root / TransactionRoot)
{
    const auto &sp(properties.staticProperties);
    if (sp.id.empty()) {
        LOGTHROW(err2, FormatError)
            << "Cannot create tile set without valid id.";
    }

    if (sp.metaLevels.delta <= 0) {
        LOGTHROW(err2, FormatError)
            << "Tile set must have positive metaLevels.delta.";
    }

    if (!create_directories(root_)) {
        // directory already exists -> fail if mode says so
        if (mode == CreateMode::failIfExists) {
            LOGTHROW(err2, TileSetAlreadyExists)
                << "Tile set at " << root_ << " already exists.";
        }
    }

    // build initial properties
    Properties p;

    // initialize create properties
    static_cast<StaticProperties&>(p) = properties.staticProperties;
    static_cast<SettableProperties&>(p)
        .merge(properties.settableProperties, properties.mask);

    // leave foat and foat size to be zero
    // leave default position

    // set templates
    p.meshTemplate = "{lod}-{easting}-{northing}.bin";
    p.textureTemplate = "{lod}-{easting}-{northing}.jpg";
    p.metaTemplate = "{lod}-{easting}-{northing}.meta";

    saveProperties_impl(p);

    // write convenience browser
    utility::write(root_ / "index.html", browser::index_html);
    utility::write(root_ / "skydome.jpg", browser::skydome_jpg);
}

FlatDriver::FlatDriver(const boost::filesystem::path &root
                       , OpenMode mode)
    : Driver(mode == OpenMode::readOnly)
    , root_(root), tmp_(root / TransactionRoot)
{
}

FlatDriver::~FlatDriver()
{
    if (txFiles_) {
        LOG(warn3) << "Active transaction on driver close; rolling back.";
        try {
            rollback_impl();
        } catch (const std::exception &e) {
            LOG(warn3)
                << "Error while trying to destroy active transaction on "
                "driver close: <" << e.what() << ">.";
        }
    }
}

Properties FlatDriver::loadProperties_impl() const
{
    // load json
    auto path(readPath(ConfigName));
    LOG(info1) << "Loading properties from " << path << ".";
    try {
        std::ifstream f;
        f.exceptions(std::ifstream::failbit | std::ifstream::badbit);
        f.open(path.string());
        Json::Reader reader;
        if (!reader.parse(f, config_)) {
            LOGTHROW(err2, FormatError)
                << "Unable to parse " << path << " config: "
                << reader.getFormattedErrorMessages() << ".";
        }
        f.close();
    } catch (const std::exception &e) {
        LOGTHROW(err2, Error)
            << "Unable to read " << path << " config: "
            << e.what() << ".";
    }

    Properties properties;
    parse(properties, config_);
    return properties;
}

void FlatDriver::saveProperties_impl(const Properties &properties)
{
    wannaWrite("save config");

    build(config_, properties);

    // save json
    auto path(writePath(ConfigName));
    LOG(info1) << "Saving properties to " << path << ".";
    try {
        std::ofstream f;
        f.exceptions(std::ifstream::failbit | std::ifstream::badbit);
        f.open(path.string());
        f.precision(15);
        Json::StyledStreamWriter().write(f, config_);
        f.close();
    } catch (const std::exception &e) {
        LOGTHROW(err2, Error)
            << "Unable to write " << path << " config: "
            << e.what() << ".";
    }
}

Driver::OStream::pointer FlatDriver::output_impl(File type)
{
    auto path(writePath(filePath(type)));
    LOG(info1) << "Saving tile index to " << path << ".";
    return std::make_shared<FileOStream>(path);
}

Driver::IStream::pointer FlatDriver::input_impl(File type) const
{
    auto path(readPath(filePath(type)));
    LOG(info1) << "Loading tile index from " << path << ".";
    return std::make_shared<FileIStream>(path);
}

Driver::OStream::pointer
FlatDriver::output_impl(const TileId tileId, TileFile type)
{
    auto path(writePath(filePath(tileId, type)));
    LOG(info1) << "Saving metatile to " << path << ".";
    return std::make_shared<FileOStream>(path);
}

Driver::IStream::pointer
FlatDriver::input_impl(const TileId tileId, TileFile type) const
{
    auto path(readPath(filePath(tileId, type)));
    LOG(info1) << "Loading metatile from " << path << ".";
    return std::make_shared<FileIStream>(path);
}

void FlatDriver::begin_impl()
{
    if (txFiles_) {
        LOGTHROW(err2, PendingTransaction)
            << "Transaction already in progress";
    }

    // remove whole tmp directory
    remove_all(tmp_);
    // create fresh tmp directory
    create_directories(tmp_);

    // begin tx
    txFiles_ = boost::in_place();
}

void FlatDriver::commit_impl()
{
    if (!txFiles_) {
        LOGTHROW(err2, PendingTransaction)
            << "No transaction in progress";
    }

    // move all files updated in transaction to from tmp to regular directory
    for (const auto &file : *txFiles_) {
        rename(tmp_ / file, root_ / file);
    }

    // remove whole tmp directory
    remove_all(tmp_);

    // no tx at all
    txFiles_ = boost::none;
}

void FlatDriver::rollback_impl()
{
    if (!txFiles_) {
        LOGTHROW(err2, PendingTransaction)
            << "No transaction in progress";
    }

    // remove whole tmp directory
    remove_all(tmp_);

    // no tx at all
    txFiles_ = boost::none;
}

fs::path FlatDriver::readPath(const fs::path &path) const
{
    if (txFiles_) {
        // we have active transaction: is file part of the tx?
        auto ftxFiles(txFiles_->find(path));
        if (ftxFiles != txFiles_->end()) {
            // yes -> tmp file
            return tmp_ / path;
        }
    }

    // regular path
    return root_ / path;
}

fs::path FlatDriver::writePath(const fs::path &path)
{

    if (!txFiles_) {
        return root_ / path;
    }

    // remember file in transaction
    txFiles_->insert(path);

    // temporary file
    return tmp_ / path;
}

} } // namespace vadstena::tilestorage
