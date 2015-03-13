#include <fstream>

#include <boost/filesystem.hpp>
#include <boost/format.hpp>
#include <boost/utility/in_place_factory.hpp>

#include "utility/path.hpp"

#include "./fsbased.hpp"
#include "./flat.hpp"
#include "../io.hpp"
#include "../error.hpp"
#include "../config.hpp"

#include "../support.hpp"

namespace vadstena { namespace tilestorage {

namespace fs = boost::filesystem;

namespace {

    const std::string ConfigName("mapConfig.json");
    const std::string TileIndexName("index.bin");
    const std::string TransactionRoot("tx");

    const std::string filePath(File type)
    {
        switch (type) {
        case File::config: return ConfigName;
        case File::tileIndex: return TileIndexName;
        }
        throw "unknown file type";
    }

    const char *extension(TileFile type)
    {
        switch (type) {
        case TileFile::meta: return "meta";
        case TileFile::mesh: return "bin";
        case TileFile::atlas: return "jpg";
        }
        throw "unknown tile file type";
    }

    fs::path filePath(const TileId &tileId, TileFile type)
    {
        return str(boost::format("%s-%07d-%07d.%s")
                   % tileId.lod % tileId.easting % tileId.northing
                   % extension(type));
    }
} // namespace

FsBasedDriver::FsBasedDriver(const boost::filesystem::path &root
                       , CreateMode mode, const CreateProperties&)
    : Driver(false)
    , root_(absolute(root)), tmp_(root_ / TransactionRoot)
    , dirCache_(root_)
{
    if (!create_directories(root_)) {
        // directory already exists -> fail if mode says so
        if (mode == CreateMode::failIfExists) {
            LOGTHROW(err2, TileSetAlreadyExists)
                << "Tile set at " << root_ << " already exists.";
        }
    }

    // write extra files
    writeExtraFiles();
}

FsBasedDriver::FsBasedDriver(const boost::filesystem::path &root
                       , OpenMode mode)
    : Driver(mode == OpenMode::readOnly)
    , root_(root), tmp_(root / TransactionRoot)
    , dirCache_(root_)
{
}

FsBasedDriver::~FsBasedDriver()
{
    if (tx_) {
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

void FsBasedDriver::writeExtraFiles()
{
    // write convenience browser
    for (const auto &file : SupportFile::files) {
        utility::write(root_ / file.first, file.second.data
                       , file.second.size);
    }
}

OStream::pointer FsBasedDriver::output_impl(File type)
{
    const auto name(filePath(type));
    const auto dir(fileDir(type, name));
    const auto path(writePath(dir, name));
    LOG(info1) << "Saving to " << path.first << ".";
    return std::make_shared<FileOStream>(path.first, path.second);
}

IStream::pointer FsBasedDriver::input_impl(File type) const
{
    const auto name(filePath(type));
    const auto dir(fileDir(type, name));
    const auto path(readPath(dir, name));
    LOG(info1) << "Loading from " << path << ".";
    return std::make_shared<FileIStream>(path);
}

OStream::pointer FsBasedDriver::output_impl(const TileId tileId, TileFile type)
{
    const auto name(filePath(tileId, type));
    const auto dir(fileDir(tileId, type, name));
    const auto path(writePath(dir, name));
    LOG(info1) << "Saving to " << path.first << ".";
    return std::make_shared<FileOStream>(path.first, path.second);
}

IStream::pointer FsBasedDriver::input_impl(const TileId tileId, TileFile type)
    const
{
    const auto name(filePath(tileId, type));
    const auto dir(fileDir(tileId, type, name));
    const auto path(readPath(dir, name));
    LOG(info1) << "Loading from " << path << ".";
    return std::make_shared<FileIStream>(path);
}

void FsBasedDriver::remove_impl(const TileId tileId, TileFile type)
{
    const auto name(filePath(tileId, type));
    const auto dir(fileDir(tileId, type, name));
    const auto path(removePath(dir, name));
    LOG(info1) << "Removing " << path << ".";

    fs::remove_all(path);
}

FileStat FsBasedDriver::stat_impl(File type) const
{
    const auto name(filePath(type));
    const auto dir(fileDir(type, name));
    const auto path(readPath(dir, name));
    LOG(info1) << "Statting from " << path << ".";
    return FileStat::stat(path);
}

FileStat FsBasedDriver::stat_impl(const TileId tileId, TileFile type) const
{
    const auto name(filePath(tileId, type));
    const auto dir(fileDir(tileId, type, name));
    const auto path(readPath(dir, name));
    LOG(info1) << "Statting from " << path << ".";
    return FileStat::stat(path);
}

void FsBasedDriver::begin_impl()
{
    if (tx_) {
        LOGTHROW(err2, PendingTransaction)
            << "Transaction already in progress";
    }

    // remove whole tmp directory
    remove_all(tmp_);
    // create fresh tmp directory
    create_directories(tmp_);

    // begin tx
    tx_ = boost::in_place(tmp_);
}

void FsBasedDriver::commit_impl()
{
    if (!tx_) {
        LOGTHROW(err2, PendingTransaction)
            << "No transaction in progress";
    }

    // move all files updated in transaction to from tmp to regular directory
    for (const auto &file : tx_->files) {
        const auto &name = file.first;
        const auto &record = file.second;
        const auto path(record.dir / name);

        if (record.removed) {
            // remove file
            remove_all(root_ / path);
        } else {
            // create directory for file
            dirCache_.create(record.dir);
            // move file
            rename(tmp_ / path, root_ / path);
        }
    }

    // remove whole tmp directory
    remove_all(tmp_);

    // no tx at all
    tx_ = boost::none;
}

void FsBasedDriver::rollback_impl()
{
    if (!tx_) {
        LOGTHROW(err2, PendingTransaction)
            << "No transaction in progress";
    }

    // remove whole tmp directory
    remove_all(tmp_);

    // no tx at all
    tx_ = boost::none;
}

void FsBasedDriver::drop_impl()
{
    if (tx_) {
        LOGTHROW(err2, PendingTransaction)
            << "Cannot drop tile set inside an active transaction.";
    }

    // remove whole tmp directory
    remove_all(root_);
}

void FsBasedDriver::update_impl()
{
    if (tx_) {
        LOGTHROW(err2, PendingTransaction)
            << "Cannot update tile set inside an active transaction.";
    }

    // write extra files (i.e. browser)
    writeExtraFiles();
}

fs::path FsBasedDriver::readPath(const fs::path &dir, const fs::path &name)
    const
{
    if (tx_) {
        // we have active transaction: is file part of the tx?
        auto ffiles(tx_->files.find(name));
        if (ffiles != tx_->files.end()) {
            // yes -> tmp file
            return tmp_ / dir / name;
        }
    }

    // regular path
    return root_ / dir / name;
}

std::pair<fs::path, FileOStream::OnClose>
FsBasedDriver::writePath(const fs::path &dir, const fs::path &name)
{
    DirCache *dirCache(&dirCache_);
    if (tx_) {
        // temporary file
        dirCache = &tx_->dirCache;
    }

    // get dir and return full path
    auto outFile(dirCache->create(dir) / name);
    if (!tx_) {
        // no destination
        return { outFile, [](bool){} };
    }

    auto tmpFile(utility::addExtension(outFile, ".tmp"));
    return {
        tmpFile
        , [this, outFile, tmpFile, name, dir] (bool success)
        {
            if (!success) {
                // failed -> remove
                LOG(warn2)
                    << "Removing failed file " << tmpFile << ".";
                fs::remove(tmpFile);
                return;
            }

            // OK -> move file to destination
            LOG(info1)
                << "Moving file " << tmpFile << " to " << outFile << ".";
            rename(tmpFile, outFile);

            // remember file in transaction
            auto res(tx_->files.insert(Tx::Files::value_type(name, dir)));
            if (res.first->second.removed) {
                // mark as existing
                res.first->second.removed = false;
            }
        }
    };
}

fs::path
FsBasedDriver::removePath(const fs::path &dir, const fs::path &name)
{
    DirCache *dirCache(&dirCache_);
    if (tx_) {
        // temporary file
        dirCache = &tx_->dirCache;
    }

    // get dir and return full path
    auto outFile(dirCache->path(dir) / name);

    // remember removal in transaction
    if (tx_) {
        auto res
            (tx_->files.insert(Tx::Files::value_type(name, { dir, true })));
        if (!res.first->second.removed) {
            // mark as removed
            res.first->second.removed = true;
        }
    }

    return outFile;
}

fs::path FsBasedDriver::DirCache::create(const fs::path &dir)
{
    if (dir.empty()) { return root_; }

    auto rdir(root_ / dir);
    if (dirs_.find(dir) == dirs_.end()) {
        // possibly non-existent dir, create
        create_directories(rdir);
        dirs_.insert(dir);
    }
    return rdir;
}

fs::path FsBasedDriver::DirCache::path(const fs::path &dir)
{
    return root_ / dir;
}

std::string FsBasedDriver::detectType_impl(const std::string &location)
{
    try {
        // try load config
        return tilestorage::loadConfig
            (fs::path(location) / filePath(File::config)).driver.type;
    } catch (const std::exception&) {}
    return {};
}

const std::string FlatDriver::help
("Filesystem-based storage driver with flat structure: all "
 "files are stored in one directory.");

} } // namespace vadstena::tilestorage

