#include "./fs.hpp"

namespace vadstena { namespace tilestorage {

Storage::pointer FileSystemStorage::create(const std::string &root)
{
    // 4.7 fails here
    // return std::make_shared<FileSystemStorage>(root, Create{});
    return pointer(new FileSystemStorage(root, Create{}));
}

Storage::pointer FileSystemStorage::open(const std::string &root
                                         , OpenMode mode)
{
    // 4.7 fails here
    // return std::make_shared<FileSystemStorage>(root, mode);
    return pointer(new FileSystemStorage(root, mode));
}

FileSystemStorage::FileSystemStorage(const std::string &root, Create)
{
    (void) root;
}

FileSystemStorage::FileSystemStorage(const std::string &root, OpenMode mode)
{
    (void) root;
    (void) mode;
}

FileSystemStorage::~FileSystemStorage()
{
}

Tile FileSystemStorage::getTile(const TileId &tileId)
{
    (void) tileId;
    return {{}, {}};
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
