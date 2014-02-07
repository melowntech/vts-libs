#include <fstream>

#include <boost/filesystem.hpp>
#include <boost/format.hpp>
#include <boost/utility/in_place_factory.hpp>

#include <opencv2/highgui/highgui.hpp>

#include "utility/streams.hpp"

#include "./flat.hpp"
#include "../json.hpp"
#include "../io.hpp"
#include "../error.hpp"
#include "../../binmesh.hpp"

#include "tilestorage/browser/index.html.hpp"
#include "tilestorage/browser/skydome.jpg.hpp"

namespace vadstena { namespace tilestorage {

namespace fs = boost::filesystem;

namespace {

    const std::string ConfigName("mapConfig.json");

    const std::string TileIndexName("index.bin");

    const std::string TransactionRoot("tx");

    fs::path filePath(const TileId &tileId, const std::string &ext)
    {
        return str(boost::format("%s-%07d-%07d.%s")
                   % tileId.lod % tileId.easting % tileId.northing % ext);
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

        virtual operator std::ostream&() override {
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

        virtual operator std::istream&() override {
            return f_;
        }

        virtual void close() override {
            f_.close();
        }

    private:
        std::ifstream f_;
    };

} // namespace

FlatDriver::FlatDriver(const boost::filesystem::path &root
                       , const CreateProperties &properties
                       , CreateMode mode)
    : Driver(false)
    , root_(root), tmp_(root / TransactionRoot)
{
    if (properties.id.empty()) {
        LOGTHROW(err2, FormatError)
            << "Cannot create tile set without valid id.";
    }

    if (properties.metaLevels.delta <= 0) {
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
    static_cast<CreateProperties&>(p) = properties;

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
    , root_(root)
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

Properties FlatDriver::loadProperties_impl()
{
    // load json
    auto path(root_ / ConfigName);
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
    auto path(root_ / ConfigName);
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

std::shared_ptr<Driver::OStream>
FlatDriver::metatileOutput_impl(const TileId tileId)
{
    auto path(writePath(filePath(tileId, "meta")));
    return std::make_shared<FileOStream>(path);
}

std::shared_ptr<Driver::IStream>
FlatDriver::metatileInput_impl(const TileId tileId)
{
    auto path(readPath(filePath(tileId, "meta")));
    return std::make_shared<FileIStream>(path);
}

std::shared_ptr<Driver::OStream> FlatDriver::tileIndexOutput_impl()
{
    auto path(root_ / TileIndexName);
    return std::make_shared<FileOStream>(path);
}

std::shared_ptr<Driver::IStream> FlatDriver::tileIndexInput_impl()
{
    auto path(root_ / TileIndexName);
    return std::make_shared<FileIStream>(path);
}

void FlatDriver::saveMesh_impl(const TileId tileId, const Mesh &mesh)
{
    writeBinaryMesh(writePath(filePath(tileId, "bin")), mesh);
}

Mesh FlatDriver::loadMesh_impl(const TileId tileId)
{
    return loadBinaryMesh(readPath(filePath(tileId, "bin")));
}

void FlatDriver::saveAtlas_impl(const TileId tileId, const Atlas &atlas
                                , short textureQuality)
{
    auto path(writePath(filePath(tileId, "jpg")));

    // TODO: check errors
    cv::imwrite(path.string().c_str(), atlas
                , { cv::IMWRITE_JPEG_QUALITY, textureQuality });
}

Atlas FlatDriver::loadAtlas_impl(const TileId tileId)
{
    auto path(readPath(filePath(tileId, "jpg")));

    auto atlas(cv::imread(path.string().c_str()));
    if (atlas.empty()) {
        LOGTHROW(err2, Error)
            << "Atlas " << tileId << " (at " << path << ") doesn't exist.";
    }
    return atlas;
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

fs::path FlatDriver::readPath(const fs::path &path)
{
    LOG(info4) << "readPath for " << path;
    if (txFiles_) {
        // we have active transaction: is file part of the tx?
        auto ftxFiles(txFiles_->find(path));
        if (ftxFiles != txFiles_->end()) {
            // yes -> tmp file
            LOG(info4) << "writePath for " << path << " tmp";
            return tmp_ / path;
        }
    }

    // regular path
    LOG(info4) << "writePath for " << path << " regular";
    return root_ / path;
}

fs::path FlatDriver::writePath(const fs::path &path)
{

    if (!txFiles_) {
        LOG(info4) << "writePath for " << path << " regular";
        return root_ / path;
    }

    // remember file in transaction
    txFiles_->insert(path);

    // temporary file
        LOG(info4) << "writePath for " << path << " tmp";
    return tmp_ / path;
}

} } // namespace vadstena::tilestorage
