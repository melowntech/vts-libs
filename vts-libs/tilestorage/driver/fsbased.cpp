#include <fstream>

#include <boost/filesystem.hpp>
#include <boost/format.hpp>
#include <boost/utility/in_place_factory.hpp>

#include "utility/streams.hpp"

#include "./flat.hpp"
#include "../io.hpp"
#include "../error.hpp"

#include "tilestorage/browser/index.html.hpp"
#include "tilestorage/browser/skydome.jpg.hpp"

namespace vadstena { namespace tilestorage {

namespace fs = boost::filesystem;

namespace {

    const std::string ConfigName("mapConfig.json");
    const std::string TileIndexName("index.bin");
    const std::string TransactionRoot("tx");

    const std::string filePath(Driver::File type)
    {
        switch (type) {
        case Driver::File::config: return ConfigName;
        case Driver::File::tileIndex: return TileIndexName;
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

        virtual std::ostream& get() UTILITY_OVERRIDE {
            return f_;
        }

        virtual void close() UTILITY_OVERRIDE {
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

        virtual std::istream& get() UTILITY_OVERRIDE {
            return f_;
        }

        virtual void close() UTILITY_OVERRIDE {
            f_.close();
        }

    private:
        utility::ifstreambuf f_;
    };

} // namespace

FsBasedDriver::FsBasedDriver(const boost::filesystem::path &root
                       , CreateMode mode)
    : Driver(false)
    , root_(root), tmp_(root / TransactionRoot)
{
    if (!create_directories(root_)) {
        // directory already exists -> fail if mode says so
        if (mode == CreateMode::failIfExists) {
            LOGTHROW(err2, TileSetAlreadyExists)
                << "Tile set at " << root_ << " already exists.";
        }
    }

    // write convenience browser
    utility::write(root_ / "index.html", browser::index_html);
    utility::write(root_ / "skydome.jpg", browser::skydome_jpg);
}

FsBasedDriver::FsBasedDriver(const boost::filesystem::path &root
                       , OpenMode mode)
    : Driver(mode == OpenMode::readOnly)
    , root_(root), tmp_(root / TransactionRoot)
{
}

FsBasedDriver::~FsBasedDriver()
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

Driver::OStream::pointer FsBasedDriver::output_impl(File type)
{
    auto path(writePath(fileDir(type), filePath(type)));
    LOG(info1) << "Saving to " << path << ".";
    return std::make_shared<FileOStream>(path);
}

Driver::IStream::pointer FsBasedDriver::input_impl(File type) const
{
    auto path(readPath(fileDir(type), filePath(type)));
    LOG(info1) << "Loading from " << path << ".";
    return std::make_shared<FileIStream>(path);
}

Driver::OStream::pointer
FsBasedDriver::output_impl(const TileId tileId, TileFile type)
{
    auto path(writePath(fileDir(tileId, type), filePath(tileId, type)));
    LOG(info1) << "Saving to " << path << ".";
    return std::make_shared<FileOStream>(path);
}

Driver::IStream::pointer
FsBasedDriver::input_impl(const TileId tileId, TileFile type) const
{
    auto path(readPath(fileDir(tileId, type), filePath(tileId, type)));
    LOG(info1) << "Loading from " << path << ".";
    return std::make_shared<FileIStream>(path);
}

void FsBasedDriver::begin_impl()
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

void FsBasedDriver::commit_impl()
{
    if (!txFiles_) {
        LOGTHROW(err2, PendingTransaction)
            << "No transaction in progress";
    }

    // move all files updated in transaction to from tmp to regular directory
    for (const auto &file : *txFiles_) {
        const auto &name = file.first;
        const auto &dir = file.second;
        // TODO: cache

        // create directory for file
        if (!dir.empty()) { create_directories(root_ / dir); }

        auto path(dir / name);
        rename(tmp_ / path, root_ / path);
    }

    // remove whole tmp directory
    remove_all(tmp_);

    // no tx at all
    txFiles_ = boost::none;
}

void FsBasedDriver::rollback_impl()
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

fs::path FsBasedDriver::readPath(const fs::path &dir, const fs::path &name)
    const
{
    if (txFiles_) {
        // we have active transaction: is file part of the tx?
        auto ftxFiles(txFiles_->find(name));
        if (ftxFiles != txFiles_->end()) {
            // yes -> tmp file
            return tmp_ / dir / name;
        }
    }

    // regular path
    return root_ / dir / name;
}

fs::path FsBasedDriver::writePath(const fs::path &dir, const fs::path &name)
{
    auto root(root_);
    if (txFiles_) {
        // remember file in transaction
        txFiles_->insert(TxFiles::value_type(name, dir));

        // temporary file
        root = tmp_;
    }

    // make directory if not empty and return full file path
    auto rdir(root / dir);
    if (!dir.empty()) { create_directories(rdir); }
    return rdir / name;
}

} } // namespace vadstena::tilestorage
