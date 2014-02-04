#include <fstream>
#include <cstring>
#include <queue>

#include <boost/format.hpp>
#include <boost/filesystem.hpp>

#include <opencv2/highgui/highgui.hpp>

#include "dbglog/dbglog.cpp"
#include "utility/binaryio.hpp"

#include "imgproc/rastermask.hpp"

#include "../binmesh.hpp"
#include "../entities.hpp"

#include "./tileop.hpp"
#include "./fs.hpp"
#include "./io.hpp"
#include "./json.hpp"

#include "tilestorage/browser/index.html.hpp"
#include "tilestorage/browser/skydome.jpg.hpp"

namespace vadstena { namespace tilestorage {

namespace fs = boost::filesystem;

namespace {

/** Extend to allow removed nodes. Pointer? Optional?
 */
typedef std::map<TileId, MetaNode> Metadata;

using imgproc::quadtree::RasterMask;

const char TILE_INDEX_IO_MAGIC[7] = { 'T', 'I', 'L', 'E', 'I', 'D', 'X' };

const char METATILE_IO_MAGIC[8] = {  'M', 'E', 'T', 'A', 'T', 'I', 'L', 'E' };

const unsigned METATILE_IO_VERSION = 1;

class TileIndex {
public:
    TileIndex() : baseTileSize_(), minLod_() {}

    TileIndex(long baseTileSize
              , Extents extents
              , LodRange lodRange
              , const TileIndex &other);

    void load(const fs::path &path);

    void save(const fs::path &path) const;

    bool exists(const TileId &tileId) const;

    void fill(const Metadata &metadata);

    Extents extents() const;

    bool empty() const;

    Lod maxLod() const;

private:
    typedef std::vector<RasterMask> Masks;

    const RasterMask* mask(Lod lod) const;

    RasterMask* mask(Lod lod);

    void fill(Lod lod, const TileIndex &other);

    long baseTileSize_;
    Point2l origin_;
    Lod minLod_;
    Masks masks_;
};

TileIndex::TileIndex(long baseTileSize
                     , Extents extents
                     , LodRange lodRange
                     , const TileIndex &other)
    : baseTileSize_(baseTileSize)
    , origin_(extents.ll)
    , minLod_(lodRange.min)
{
    // include old definition if non-empty
    if (!other.empty()) {
        // something present in on-disk data
        extents = unite(extents, other.extents());
        origin_ = extents.ll;

        if (lodRange.min > other.minLod_) {
            // update min lod
            minLod_ = lodRange.min  = other.minLod_;
        }

        if (lodRange.max < other.maxLod()) {
            // update max lod
            lodRange.max = other.maxLod();
        }
    }

    // maximal tile size (at lowest lod)
    auto ts(tileSize(baseTileSize, lodRange.min));
    // extents size
    auto es(size(extents));

    // size in tiles
    math::Size2i tiling((es.width + ts - 1) / ts
                        , (es.height + ts - 1) / ts);

    // create raster mask for all lods
    for (auto lod : lodRange) {
        masks_.emplace_back(tiling, RasterMask::EMPTY);

        // half the tile
        ts /= 2;

        // double tile count
        tiling.width *= 2;
        tiling.height *= 2;

        // fill in old data
        fill(lod, other);
    }
}

inline bool TileIndex::empty() const
{
    return masks_.empty();
}

inline Lod TileIndex::maxLod() const
{
    if (masks_.empty()) { return minLod_; }
    return minLod_ + masks_.size();
}

inline Extents TileIndex::extents() const
{
    if (masks_.empty()) { return {}; }

    auto size(masks_.front().size());
    auto ts(tileSize(baseTileSize_, minLod_));

    return { origin_(0), origin_(1)
            , origin_(0) + ts * size.width
            , origin_(1) + ts * size.height };
}

inline const RasterMask* TileIndex::mask(Lod lod) const
{
    auto idx(lod - minLod_);
    if ((idx < 0) || (idx > int(masks_.size()))) {
        return nullptr;
    }

    // get mask
    return &masks_[idx];
}

inline RasterMask* TileIndex::mask(Lod lod)
{
    auto idx(lod - minLod_);
    if ((idx < 0) || (idx > int(masks_.size()))) {
        return nullptr;
    }

    // get mask
    return &masks_[idx];
}

bool TileIndex::exists(const TileId &tileId) const
{
    const auto *m(mask(tileId.lod));
    if (!m) { return false; }

    // tile size
    auto ts(tileSize(baseTileSize_, tileId.lod));

    // query mask
    return m->get((tileId.easting - origin_(0)) / ts
                  , (tileId.northing - origin_(1)) / ts);
}

void TileIndex::fill(const Metadata &metadata)
{
    for (const auto &node : metadata) {
        const auto &tileId(node.first);

        auto *m(mask(tileId.lod));
        if (!m) {
            // lod out of interest
            continue;
        }

        // tile size
        auto ts(tileSize(baseTileSize_, tileId.lod));

        // set/unset tile presence
        m->set((tileId.easting - origin_(0)) / ts
               , (tileId.northing - origin_(1)) / ts
               , node.second.exists());
    }
}

void TileIndex::fill(Lod lod, const TileIndex &other)
{
    // find old and new masks
    const auto *oldMask(other.mask(lod));
    if (!oldMask) { return; }

    auto *newMask(mask(lod));
    if (!newMask) { return; }

    // calculate origin difference in tiles at given lod
    auto ts(tileSize(baseTileSize_, lod));

    math::Size2 diff((origin_(0) - other.origin_(0)) / ts
                     , (origin_(1) - other.origin_(1)) / ts);

    auto size(oldMask->dims());
    for (int j(0); j < size.height; ++j) {
        for (int i(0); i < size.width; ++i) {
            if (oldMask->get(i, j)) {
                newMask->set(diff.width + i, diff.height + j);
            }
        }
    }
}

void TileIndex::load(const fs::path &path)
{
    using utility::binaryio::read;

    std::ifstream f;
    f.exceptions(std::ifstream::failbit | std::ifstream::badbit);
    f.open(path.string(), std::ifstream::in | std::ifstream::binary);

    char magic[7];
    read(f, magic);

    if (std::memcmp(magic, TILE_INDEX_IO_MAGIC,
                    sizeof(TILE_INDEX_IO_MAGIC))) {
        LOGTHROW(err2, Error)
            << "TileIndex has wrong magic.";
    }

    uint8_t reserved1;
    read(f, reserved1); // reserved

    int64_t o;
    read(f, o);
    origin_(0) = o;
    read(f, o);
    origin_(1) = o;

    int16_t minLod, size;
    read(f, minLod);
    read(f, size);
    minLod_ = minLod;

    masks_.resize(size);

    for (auto &mask : masks_) {
        mask.load(f);
    }
}

void TileIndex::save(const fs::path &path) const
{
    using utility::binaryio::write;

    std::ofstream f;
    f.exceptions(std::ifstream::failbit | std::ifstream::badbit);
    f.open(path.string(), std::ifstream::out | std::ifstream::binary);

    write(f, TILE_INDEX_IO_MAGIC); // 7 bytes
    write(f, uint8_t(0)); // reserved

    write(f, int64_t(origin_(0)));
    write(f, int64_t(origin_(1)));

    write(f, int16_t(minLod_));
    write(f, int16_t(masks_.size()));

    // save lod-mask mapping
    for (const auto &mask : masks_) {
        mask.dump(f);
    }

    f.close();
}

struct MetatileDef {
    TileId id;
    Lod end;

    MetatileDef(const TileId id, Lod end)
        : id(id), end(end)
    {}

    bool bottom() const { return (id.lod + 1) >= end; }

    typedef std::queue<MetatileDef> queue;
};

} // namespace

struct FileSystemStorage::Detail {
    boost::filesystem::path root;
    bool readOnly;

    // properties
    Json::Value config;     // parsed config as JSON tree
    Properties savedProperties;  // properties as are on disk
    Properties properties;       // current properties
    bool propertiesChanged; // marks whether properties have been changed

    TileIndex tileIndex;    // tile index that reflects state on the disk

    mutable Extents extents;      // extents covered by tiles
    mutable LodRange lodRange;    // covered lod range
    mutable Metadata metadata;    // all metadata as in-memory structure
    mutable bool metadataChanged; // marks whether metadata have been changed

    Detail(const boost::filesystem::path &root, bool readOnly)
        : root(root), readOnly(readOnly), propertiesChanged(false)
        , metadataChanged(false)
    {}

    void wannaWrite(const std::string &what) const;

    void check(const TileId &tileId) const;

    void loadConfig();

    void saveConfig();

    void saveMetadata();

    void loadTileIndex();

    MetaNode* loadMetatile(const TileId &tileId) const;

    MetaNode* findMetaNode(const TileId &tileId) const;

    void setMetaNode(const TileId &tileId, const MetaNode& metanode);

    void setMetadata(const TileId &tileId, const TileMetadata& metadata);

    MetaNode& createVirtualMetaNode(const TileId &tileId);

    bool isFoat(const TileId &tileId) const;

    void updateZbox(const TileId &tileId, MetaNode &metanode);

    void flush();

    void saveMetatiles() const;

    void saveMetatile(MetatileDef::queue &subtrees
                      , const MetatileDef &tile) const;

    void saveMetatileTree(MetatileDef::queue &subtrees, std::ostream &f
                          , const MetatileDef &tile) const;
};

namespace {

const std::string ConfigName("mapConfig.json");

const std::string TileIndexName("index.bin");

fs::path filePath(const TileId &tileId, const std::string &ext)
{
    return str(boost::format("%s-%07d-%07d.%s")
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

    parse(properties, config);
    savedProperties = properties;
}

void FileSystemStorage::Detail::saveConfig()
{
    wannaWrite("save config");

    build(config, properties);

    // save json
    auto path(root / ConfigName);
    try {
        std::ofstream f;
        f.exceptions(std::ifstream::failbit | std::ifstream::badbit);
        f.open(path.string());
        f.precision(15);
        Json::StyledStreamWriter().write(f, config);
        f.close();
    } catch (const std::exception &e) {
        LOGTHROW(err2, Error)
            << "Unable to write " << path << " config: "
            << e.what() << ".";
    }

    // done; remember saved properties and go on
    savedProperties = properties;
    propertiesChanged = false;
}

void FileSystemStorage::Detail::saveMetadata()
{
    wannaWrite("save metadata");

    // create tile index (initialize with existing one)
    TileIndex ti(properties.baseTileSize, extents, lodRange
                 , tileIndex);

    // fill in new metadata
    ti.fill(metadata);

    // save
    try {
        ti.save(root / TileIndexName);
    } catch (const std::exception &e) {
        LOGTHROW(err2, Error)
            << "Unable to write tile index at " << (root / TileIndexName)
            << ": " << e.what() << ".";
    }

    // cool, we have new tile index
    tileIndex = ti;

    // TODO: we have to load all metatiles present on the disk to proceed
    if (metadata.empty()) {
        // no tile, we should invalidate foat
        properties.foat = {};
        LOG(info2) << "New foat is " << properties.foat << ".";
    }

    // well, dump metatiles now
    saveMetatiles();

    // saved => no change
    metadataChanged = false;
}

void FileSystemStorage::Detail::loadTileIndex()
{
    try {
        tileIndex.load(root / TileIndexName);
    } catch (const std::exception &e) {
        LOGTHROW(err2, Error)
            << "Unable to read tile index at " << (root / TileIndexName)
            << ": " << e.what() << ".";
    }

    // new extents
    extents = tileIndex.extents();
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

    // grow extents
    if (!area(extents)) {
        // invalid extents, add first tile!
        extents = tileExtents(properties, tileId);
        // initial lod range
        lodRange.min = lodRange.max = tileId.lod;
    } else {
        // add tile

        // update extents
        extents = unite(extents, tileExtents(properties, tileId));

        // update lod range
        if (tileId.lod < lodRange.min) {
            lodRange.min = tileId.lod;
        } else if (tileId.lod > lodRange.max) {
            lodRange.max = tileId.lod;
        }
    }

    // layout updated -> update zboxes up the tree
    updateZbox(tileId, res.first->second);

    metadataChanged = true;
}

MetaNode&
FileSystemStorage::Detail::createVirtualMetaNode(const TileId &tileId)
{
    // this ensures that we have old metanode in memory
    auto *md(findMetaNode(tileId));
    if (!md) {
        // no node (real or virtual), create virtual node
        md = &(metadata.insert(Metadata::value_type(tileId, MetaNode()))
               .first->second);
    }

    metadataChanged = true;

    // ok
    return *md;
}

MetaNode* FileSystemStorage::Detail::loadMetatile(const TileId &tileId)
    const
{
    // TODO: implement me
    (void) tileId;
    return nullptr;

    // NB: this call loads whole metatile file where tileId lives; only nodes
    // that are not present in this->metadata are inserted there
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

void FileSystemStorage::Detail::check(const TileId &tileId) const
{
    auto ts(tileSize(properties, tileId.lod));

    auto aligned(fromAlignment(properties, tileId));
    if ((aligned.easting % ts) || (aligned.northing % ts)) {
        LOGTHROW(err2, NoSuchTile)
            << "Misaligned tile at " << tileId << " cannot exist.";
    }
}

void FileSystemStorage::Detail::updateZbox(const TileId &tileId
                                           , MetaNode &metanode)
{
    // process all 4 children
    for (const auto &childId : children(properties.baseTileSize, tileId)) {
        if (auto *node = findMetaNode(childId)) {
            metanode.zmin = std::min(metanode.zmin, node->zmin);
            metanode.zmax = std::max(metanode.zmax, node->zmax);
        }
    }

    // this tile is (current) foat -> no parent can ever exist (until real tile
    // is added)
    if (isFoat(tileId)) {
        // we reached foat tile => way up
        if (tileId != properties.foat) {
            properties.foat = tileId;
            properties.foatSize = tileSize(properties, tileId.lod);
            propertiesChanged = true;
            LOG(info2) << "New foat is " << properties.foat << ".";
        }
        return;
    }

    auto parentId(parent(properties.alignment
                         , properties.baseTileSize
                         , tileId));
    if (auto *parentNode = findMetaNode(parentId)) {
        updateZbox(parentId, *parentNode);
    } else {
        // there is no parent present in the tree; process freshly generated
        // parent
        updateZbox(parentId, createVirtualMetaNode(parentId));
    }
}

bool FileSystemStorage::Detail::isFoat(const TileId &tileId) const
{
    if (extents.ll(0) < tileId.easting) { return false; }
    if (extents.ll(1) < tileId.northing) { return false; }

    auto ts(tileSize(properties.baseTileSize, tileId.lod));

    if (extents.ur(0) > (tileId.easting + ts)) { return false; }
    if (extents.ur(1) > (tileId.northing + ts)) { return false; }

    return true;
}

void FileSystemStorage::Detail::flush()
{
    if (readOnly) { return; }
    LOG(info2) << "Flushing storage at path: " << root;

    // force metadata save
    if (metadataChanged) {
        saveMetadata();
    }

    // force config save
    if (propertiesChanged) {
        saveConfig();
    }
}

void FileSystemStorage::Detail::saveMetatileTree(MetatileDef::queue &subtrees
                                                 , std::ostream &f
                                                 , const MetatileDef &tile)
    const
{
    using utility::binaryio::write;

    auto bottom(tile.bottom());

    LOG(info2) << "dumping " << tile.id << ", " << tile.end
               << ", bottom: " << bottom
               << ".";

    const auto *node(findMetaNode(tile.id));
    if (!node) {
        LOGTHROW(err2, Error)
            << "Can't find metanode for tile " << tile.id;
    }

    node->dump(f);

    std::uint8_t childFlags(0);
    std::uint8_t mask(1);
    auto childrenIds(children(properties.baseTileSize, tile.id));

    for (auto &childId : childrenIds) {
        if (findMetaNode(childId)) {
            childFlags |= mask;
        }
        mask <<= 1;
    }

    if (!bottom) {
        // children are local
        childFlags |= (childFlags << 4);
    }
    write(f, childFlags);

    // either dump 4 subnodes now or remember them in the subtrees
    mask = 1;
    for (const auto &childId : childrenIds) {
        if ((childFlags & mask)) {
            if (bottom) {
                // we are at the bottom of the metatile; remember subtree
                subtrees.emplace
                    (childId, deltaDown(properties.metaLevels, childId.lod));
            } else {
                // save subtree in this tile
                saveMetatileTree(subtrees, f
                                 , { childId, deltaDown(properties.metaLevels
                                                        , childId.lod) });
            }
        }
        mask <<= 1;
    }
}

/** Save metatile at given tile: subtree from tile'slod until end lod is
 * reached
 */
void FileSystemStorage::Detail::saveMetatile(MetatileDef::queue &subtrees
                                             , const MetatileDef &tile)
    const
{
    using utility::binaryio::write;

    auto path(root / filePath(tile.id, "meta"));

    LOG(info2) << "saving metatile to " << path << ".";

    utility::ofstreambuf f;
    f.exceptions(std::ios::badbit | std::ios::failbit);
    f.open(path.string(), std::ios_base::out | std::ios_base::trunc);

    write(static_cast<std::ostream&>(f), METATILE_IO_MAGIC);
    write(f, uint32_t(METATILE_IO_VERSION));

    saveMetatileTree(subtrees, f, tile);

    f.close();
}

void FileSystemStorage::Detail::saveMetatiles() const
{
    // initialize subtrees with foat
    MetatileDef::queue subtrees;
    subtrees.emplace
        (properties.foat
         , deltaDown(properties.metaLevels, properties.foat.lod));

    while (!subtrees.empty()) {
        saveMetatile(subtrees, subtrees.front());
        subtrees.pop();
    }
}

inline void FileSystemStorage::Detail::wannaWrite(const std::string &what)
    const
{
    if (readOnly) {
        LOGTHROW(err2, Error)
            << "Cannot " << what << ": storage is read-only.";
    }
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

    // write convenience browser

    utility::write(detail().root / "index.html", browser::index_html);
    utility::write(detail().root / "skydome.jpg", browser::skydome_jpg);
}

FileSystemStorage::FileSystemStorage(const std::string &root, OpenMode mode)
    : detail_(new Detail(root, mode == OpenMode::readOnly))
{
    detail().loadConfig();
    detail().loadTileIndex();
}

FileSystemStorage::~FileSystemStorage()
{
    // ?
}

void FileSystemStorage::flush_impl()
{
    detail().flush();
}

Tile FileSystemStorage::getTile_impl(const TileId &tileId) const
{
    detail().check(tileId);

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
    detail().wannaWrite("set tile");

    detail().check(tileId);

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
    detail().wannaWrite("set tile metadata");

    detail().check(tileId);

    detail().setMetadata(tileId, metadata);
}

bool FileSystemStorage::tileExists_impl(const TileId &tileId) const
{
    // have look to in-memory data
    const auto &metadata(detail().metadata);
    auto fmetadata(metadata.find(tileId));
    if (fmetadata != metadata.end()) {
        return true;
    }

    // try tileIndex
    return detail().tileIndex.exists(tileId);
}

Properties FileSystemStorage::getProperties_impl() const
{
    return detail().properties;
}

Properties
FileSystemStorage::setProperties_impl(const SettableProperties &properties
                                      , int mask)
{
    detail().wannaWrite("set properties");

    // merge in new properties
    if (detail().properties.merge(properties, mask)) {
        detail().propertiesChanged = true;
    }
    return detail().properties;
}

} } // namespace vadstena::tilestorage
