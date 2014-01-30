#include <fstream>

#include <boost/format.hpp>
#include <boost/filesystem.hpp>

#include <opencv2/highgui/highgui.hpp>

#include "dbglog/dbglog.cpp"
#include "jsoncpp/json.hpp"
#include "jsoncpp/as.hpp"

#include "../binmesh.hpp"

#include "./fs.hpp"

namespace vadstena { namespace tilestorage {

namespace fs = boost::filesystem;

struct FileSystemStorage::Detail {
    boost::filesystem::path root;
    bool readOnly;
    Json::Value config;
    Properties properties;

    Detail(const boost::filesystem::path &root, bool readOnly)
        : root(root), readOnly(readOnly) {}
};

namespace {

const std::string ConfigName("mapConfig.json");

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

    // look forward by default
    p.defaultOrientation = { 0, -90, 0 };

    // set templates
    p.meshTemplate = "{lod}-{easting}-{northing}.bin";
    p.textureTemplate = "{lod}-{easting}-{northing}.jpg";
    p.metaTemplate = "{lod}-{easting}-{northing}.meta";

    saveConfig();
}

FileSystemStorage::FileSystemStorage(const std::string &root, OpenMode mode)
    : detail_(new Detail(root, mode == OpenMode::readOnly))
{
    loadConfig();
}

FileSystemStorage::~FileSystemStorage()
{
    // ?
}

void FileSystemStorage::flush_impl()
{
    if (detail().readOnly) { return; }
    // TODO: flush here
    LOG(info2) << "Flushing storage at path: " << detail().root;

    // force config save
    saveConfig();
}

Tile FileSystemStorage::getTile_impl(const TileId &tileId)
{
    return {
        loadBinaryMesh(detail().root / filePath(tileId, "bin"))
        , loadTexture(detail().root / filePath(tileId, "jpg"))
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
    return detail().properties;
}

Properties
FileSystemStorage::setProperties_impl(const SettableProperties &properties
                                      , int mask)
{
    (void) properties;
    (void) mask;

    return {};
}

void FileSystemStorage::loadConfig()
{
    // load json
    auto &config(detail().config);

    auto path(detail().root / ConfigName);
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

    auto &p(detail().properties);

    const auto &foat(config["foat"]);
    p.foat.lod = Json::as<Lod>(foat[0]);
    p.foat.easting = Json::as<long>(foat[1]);
    p.foat.northing = Json::as<long>(foat[2]);

    const auto &meta(config["meta"]);
    p.metaLevels.lod = Json::as<Lod>(meta[0]);
    p.metaLevels.delta = Json::as<Lod>(meta[1]);

    p.meshTemplate = Json::as<std::string>(config["meshTemplate"]);
    p.textureTemplate = Json::as<std::string>(config["textureTemplate"]);
    p.metaTemplate = Json::as<std::string>(config["metaTemplate"]);

    const auto &defaultPosition(config["defaultPosition"]);
    p.defaultPosition(0) = Json::as<double>(defaultPosition[0]);
    p.defaultPosition(1) = Json::as<double>(defaultPosition[1]);
    p.defaultPosition(2) = Json::as<double>(defaultPosition[2]);

    const auto &defaultOrientation(config["defaultOrientation"]);
    p.defaultOrientation(0) = Json::as<double>(defaultOrientation[0]);
    p.defaultOrientation(1) = Json::as<double>(defaultOrientation[1]);
    p.defaultOrientation(2) = Json::as<double>(defaultOrientation[2]);
}

void FileSystemStorage::saveConfig()
{
    if (detail().readOnly) {
        LOGTHROW(err2, Error) << "Storage is read-only.";
    }

    // build json
    auto &p(detail().properties);
    auto &config(detail().config);

    auto &foat(config["foat"] = Json::Value(Json::arrayValue));
    foat.append(Json::UInt64(p.foat.lod));
    foat.append(Json::UInt64(p.foat.easting));
    foat.append(Json::UInt64(p.foat.northing));
    foat.append(Json::UInt64(p.foatSize));

    auto &meta(config["meta"] = Json::Value(Json::arrayValue));
    meta.append(Json::UInt64(p.metaLevels.lod));
    meta.append(Json::UInt64(p.metaLevels.delta));

    config["meshTemplate"] = p.meshTemplate;
    config["textureTemplate"] = p.textureTemplate;
    config["metaTemplate"] = p.metaTemplate;

    auto &defaultPosition
        (config["defaultPosition"] = Json::Value(Json::arrayValue));
    defaultPosition.append(p.defaultPosition(0));
    defaultPosition.append(p.defaultPosition(1));
    defaultPosition.append(p.defaultPosition(2));

    auto &defaultOrientation
        (config["defaultOrientation"] = Json::Value(Json::arrayValue));
    defaultOrientation.append(p.defaultOrientation(0));
    defaultOrientation.append(p.defaultOrientation(1));
    defaultOrientation.append(p.defaultOrientation(2));

    // save json
    auto path(detail().root / ConfigName);
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
}

} } // namespace vadstena::tilestorage
