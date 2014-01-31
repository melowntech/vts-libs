#include <fstream>
#include <cstring>

#include <boost/format.hpp>
#include <boost/filesystem.hpp>

#include <opencv2/highgui/highgui.hpp>

#include "dbglog/dbglog.cpp"

#include "../binmesh.hpp"

#include "./fs.hpp"
#include "./io.hpp"
#include "./json.hpp"

namespace vadstena { namespace tilestorage {

namespace fs = boost::filesystem;

typedef std::map<TileId, MetaNode> Metadata;

struct FileSystemStorage::Detail {
    boost::filesystem::path root;
    bool readOnly;

    // properties
    Json::Value config;     // parsed config as JSON tree
    Properties properties;  // properties
    bool propertiesChanged; // marks whether properties have been changed

    mutable Metadata metadata;    // all metadata as in-memory structure
    mutable bool metadataChanged; // marks whether metadata have been changed

    Detail(const boost::filesystem::path &root, bool readOnly)
        : root(root), readOnly(readOnly), propertiesChanged(false)
        , metadataChanged(false)
    {}

    void loadConfig();

    void saveConfig();

    void saveMetadata();

    MetaNode* loadMetatile(const TileId &tileId) const;

    MetaNode* findMetaNode(const TileId &tileId) const;

    void setMetaNode(const TileId &tileId, const MetaNode& metanode);

    void setMetadata(const TileId &tileId, const TileMetadata& metadata);
};

namespace {

const std::string ConfigName("mapConfig.json");

fs::path filePath(const TileId &tileId, const std::string &ext)
{
    return str(boost::format("%s-%s-%s.%s")
               % tileId.lod % tileId.easting % tileId.northing % ext);
}

cv::Mat loadAtlas(const fs::path &path)
{
    auto atlas(cv::imread(path.string().c_str()));
    if (atlas.empty()) {
        LOGTHROW(err2, Error)
            << "Atlas at " << path << " doesn't exist.";
    }
    return atlas;
}

void saveAtlas(const fs::path &path, const cv::Mat &atlas
               , short jpegQuality)
{
    // TODO: check errors
    // TODO: quality?
    cv::imwrite(path.string().c_str(), atlas
                , { cv::IMWRITE_JPEG_QUALITY, jpegQuality });
}

} // namespace

void FileSystemStorage::Detail::loadConfig()
{
    // load json
    auto path(root / ConfigName);
    try {
        std::ifstream f;
        f.exceptions(std::ifstream::failbit | std::ifstream::badbit);
        f.open(path.string());
        Json::Reader reader;
        if (!reader.parse(f, config)) {
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

    // TODO: check errors

    parse(properties, config);
}

void FileSystemStorage::Detail::saveConfig()
{
    if (readOnly) {
        LOGTHROW(err2, Error) << "Storage is read-only.";
    }

    build(config, properties);

    // save json
    auto path(root / ConfigName);
    try {
        std::ofstream f;
        f.exceptions(std::ifstream::failbit | std::ifstream::badbit);
        f.open(path.string());
        Json::StyledStreamWriter().write(f, config);
        f.close();
    } catch (const std::exception &e) {
        LOGTHROW(err2, Error)
            << "Unable to write " << path << " config: "
            << e.what() << ".";
    }

    // done
    propertiesChanged = false;
}

void FileSystemStorage::Detail::saveMetadata()
{
    if (readOnly) {
        LOGTHROW(err2, Error) << "Storage is read-only.";
    }

    metadataChanged = false;
}

MetaNode* FileSystemStorage::Detail::findMetaNode(const TileId &tileId)
    const
{
    auto fmetadata(metadata.find(tileId));
    if (fmetadata == metadata.end()) {
        // not found in memory -> load from disk
        return loadMetatile(tileId);
    }

    return &fmetadata->second;
}

void FileSystemStorage::Detail::setMetaNode(const TileId &tileId
                                            , const MetaNode& metanode)
{
    // this ensures that we have old metanode in memory
    findMetaNode(tileId);

    auto res(metadata.insert(Metadata::value_type(tileId, metanode)));
    if (!res.second) {
        res.first->second = metanode;
    }
    metadataChanged = true;
}

MetaNode* FileSystemStorage::Detail::loadMetatile(const TileId &tileId)
    const
{
    // TODO: implement me
    (void) tileId;
    return nullptr;

    // NB: this call loads whole metatile file where tileId lives; only nodes
    // that are not present in this->metadata are inserted there

#if 0
    // find out lod to look for

    // calculate number of lods from reference lod
    auto diff(properties.metaLevels.lod - tileId.lod);
    // calculate steps from reference lod
    auto steps(diff / properties.metaLevels.delta);
    if ((steps < 0) && (-diff % properties.metaLevels.delta)) {
        // negative and not multiple of delta -> one more step up
        --steps;
    }

    Lod mtLod(properties.metaLevels.lod
              + (steps * properties.metaLevels.delta));
#endif
}

void FileSystemStorage::Detail::setMetadata(const TileId &tileId
                                            , const TileMetadata& metadata)
{
    // this ensures that we have old metanode in memory
    auto metanode(findMetaNode(tileId));
    if (!metanode) {
        LOGTHROW(err2, NoSuchTile)
            << "There is no tile at " << tileId << ".";
    }

    // assign new metadata
    static_cast<TileMetadata&>(*metanode) = metadata;
}

// storage itself

FileSystemStorage::FileSystemStorage(const std::string &root
                                     , const CreateProperties &properties
                                     , CreateMode mode)
    : detail_(new Detail(root, false))
{
    if (!create_directories(detail().root)) {
        // directory already exists -> fail if mode says so
        if (mode == CreateMode::failIfExists) {
            LOGTHROW(err2, StorageAlreadyExists)
                << "Storage at " << detail().root << " already exits.";
        }
    }

    // build initial properties

    // initialize create properties
    auto &p(detail().properties);
    static_cast<CreateProperties&>(p) = properties;

    // leave foat and foat size to be zero
    // leave default position

    // set templates
    p.meshTemplate = "{lod}-{easting}-{northing}.bin";
    p.textureTemplate = "{lod}-{easting}-{northing}.jpg";
    p.metaTemplate = "{lod}-{easting}-{northing}.meta";

    detail().propertiesChanged = true;
    flush();
}

FileSystemStorage::FileSystemStorage(const std::string &root, OpenMode mode)
    : detail_(new Detail(root, mode == OpenMode::readOnly))
{
    detail().loadConfig();
}

FileSystemStorage::~FileSystemStorage()
{
    // ?
}

void FileSystemStorage::flush_impl()
{
    if (detail().readOnly) { return; }
    LOG(info2) << "Flushing storage at path: " << detail().root;

    // force config save
    if (detail().propertiesChanged) {
        detail().saveConfig();
    }

    // force metadata save
    if (detail().metadataChanged) {
        detail().saveMetadata();
    }
}

Tile FileSystemStorage::getTile_impl(const TileId &tileId) const
{
    auto md(detail().findMetaNode(tileId));
    if (!md) {
        LOGTHROW(err2, NoSuchTile)
            << "There is no tile at " << tileId << ".";
    }

    return {
        loadBinaryMesh(detail().root / filePath(tileId, "bin"))
        , loadAtlas(detail().root / filePath(tileId, "jpg"))
        , *md
    };
}

void FileSystemStorage::setTile_impl(const TileId &tileId, const Mesh &mesh
                                     , const Atlas &atlas
                                     , const TileMetadata *metadata)
{
    if (detail().readOnly) {
        LOGTHROW(err2, Error) << "Storage is read-only.";
    }

    writeBinaryMesh(detail().root / filePath(tileId, "bin"), mesh);
    saveAtlas(detail().root / filePath(tileId, "jpg"), atlas
              , detail().properties.textureQuality);

    // create new metadata
    MetaNode metanode;

    // copy extra metadata
    if (metadata) {
        static_cast<TileMetadata&>(metanode) = *metadata;
    }

    // calculate dependent metadata
    metanode.calcParams(mesh, { atlas.cols, atlas.rows });

    // remember new metanode
    detail().setMetaNode(tileId, metanode);
}

void FileSystemStorage::setMetadata_impl(const TileId &tileId
                                         , const TileMetadata &metadata)
{
    if (detail().readOnly) {
        LOGTHROW(err2, Error) << "Storage is read-only.";
    }

    detail().setMetadata(tileId, metadata);
}

bool FileSystemStorage::tileExists_impl(const TileId &tileId) const
{
    (void) tileId;
    return false;
}

Properties FileSystemStorage::getProperties_impl() const
{
    return detail().properties;
}

Properties
FileSystemStorage::setProperties_impl(const SettableProperties &properties
                                      , int mask)
{
    if (detail().readOnly) {
        LOGTHROW(err2, Error) << "Storage is read-only.";
    }

    // merge in new properties
    if (detail().properties.merge(properties, mask)) {
        detail().propertiesChanged = true;
    }
    return detail().properties;
}

} } // namespace vadstena::tilestorage
