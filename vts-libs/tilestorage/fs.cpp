#include <boost/format.hpp>

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
                                     , const Properties &properties
                                     , CreateMode mode)
    : root_(root)
{
    (void) root;
    (void) properties;
    (void) mode;
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

void FileSystemStorage::flush()
{
    // TODO: flush here
    LOG(info3) << "Flushing storage at file:" << root_;
}

Tile FileSystemStorage::getTile(const TileId &tileId)
{
    return {
        loadBinaryMesh(root_ / filePath(tileId, "bin"))
        , loadTexture(root_ / filePath(tileId, "jpg"))
    };
}

void FileSystemStorage::setTile(const TileId &tileId, const Mesh &mesh
                                , const Atlas &atlas)
{
    (void) tileId;
    (void) mesh;
    (void) atlas;
}

MetaNode FileSystemStorage::getMetaData(const TileId &tileId)
{
    (void) tileId;
    return {};
}

void FileSystemStorage::setMetaData(const TileId &tileId, const MetaNode &meta)
{
    (void) tileId;
    (void) meta;
}

bool FileSystemStorage::tileExists(const TileId &tileId)
{
    (void) tileId;
    return false;
}

Properties FileSystemStorage::getProperties()
{
    return {{{}, {}, {}}, {}, {}, {}};
}

void FileSystemStorage::setProperties(const Properties &properties)
{
    (void) properties;
}

} } // namespace vadstena::tilestorage
