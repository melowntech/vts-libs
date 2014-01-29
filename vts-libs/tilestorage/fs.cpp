#include <boost/format.hpp>
#include <boost/filesystem.hpp>

#include <opencv2/highgui/highgui.hpp>

#include "dbglog/dbglog.cpp"

#include "../binmesh.hpp"

#include "./fs.hpp"

namespace vadstena { namespace tilestorage {

namespace fs = boost::filesystem;

namespace {

fs::path filePath(const TileId &tileId, const std::string &ext)
{
    return str(boost::format("%s-%s-%s.%s")
               % tileId.lod % tileId.easting % tileId.northing % ext);
}

cv::Mat loadTexture(const fs::path &path)
{
    // TODO: check errors
    return cv::imread(path.string().c_str());
}

} // namespace

FileSystemStorage::FileSystemStorage(const std::string &root
                                     , const CreateProperties &properties
                                     , CreateMode mode)
    : root_(root)
{
    (void) root;
    (void) properties;
    (void) mode;

    if (!create_directories(root_)) {
        // directory already exists -> fail if mode says so
        if (mode == CreateMode::failIfExists) {
            LOGTHROW(err2, StorageAlreadyExists)
                << "Storage at " << root_ << " already exits.";
        }
    }
}

FileSystemStorage::FileSystemStorage(const std::string &root, OpenMode mode)
    : root_(root)
{
    (void) root;
    (void) mode;
}

FileSystemStorage::~FileSystemStorage()
{
}

void FileSystemStorage::flush_impl()
{
    // TODO: flush here
    LOG(info2) << "Flushing storage at path: " << root_;
}

Tile FileSystemStorage::getTile_impl(const TileId &tileId)
{
    return {
        loadBinaryMesh(root_ / filePath(tileId, "bin"))
        , loadTexture(root_ / filePath(tileId, "jpg"))
    };
}

void FileSystemStorage::setTile_impl(const TileId &tileId, const Mesh &mesh
                                , const Atlas &atlas)
{
    (void) tileId;
    (void) mesh;
    (void) atlas;
}

MetaNode FileSystemStorage::getMetaData_impl(const TileId &tileId)
{
    (void) tileId;
    return {};
}

void FileSystemStorage::setMetaData_impl(const TileId &tileId
                                         , const MetaNode &meta)
{
    (void) tileId;
    (void) meta;
}

bool FileSystemStorage::tileExists_impl(const TileId &tileId)
{
    (void) tileId;
    return false;
}

Properties FileSystemStorage::getProperties_impl()
{
    return {};
}

Properties
FileSystemStorage::setProperties_impl(const SettableProperties &properties
                                      , int mask)
{
    (void) properties;
    (void) mask;

    return {};
}

} } // namespace vadstena::tilestorage
