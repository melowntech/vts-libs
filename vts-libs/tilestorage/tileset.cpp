#include <boost/format.hpp>
#include <boost/utility/in_place_factory.hpp>

#include <opencv2/highgui/highgui.hpp>

#include "dbglog/dbglog.hpp"
#include "utility/binaryio.hpp"

#include "../binmesh.hpp"
#include "../tilestorage.hpp"
#include "./tileindex.hpp"
#include "./io.hpp"
#include "./tileop.hpp"
#include "./json.hpp"
#include "./tileset-detail.hpp"
#include "./tileset-advanced.hpp"
#include "./metatile.hpp"
#include "./merge.hpp"

// #define VADSTENA_LIBS_DUMP_TILEINDEX 1

namespace vadstena { namespace tilestorage {

namespace {

Atlas loadAtlas(const IStream::pointer &is)
{
    using utility::binaryio::read;
    auto& s(is->get());
    auto size(s.seekg(0, std::ios_base::end).tellg());
    s.seekg(0);
    std::vector<unsigned char> buf;
    buf.resize(size);
    read(s, buf.data(), buf.size());

    auto atlas(cv::imdecode(buf, CV_LOAD_IMAGE_COLOR));
    is->close();
    return atlas;
}

void saveAtlas(const OStream::pointer &os, const Atlas &atlas
               , short textureQuality)
{
    using utility::binaryio::write;
    std::vector<unsigned char> buf;
    cv::imencode(".jpg", atlas, buf
                 , { cv::IMWRITE_JPEG_QUALITY, textureQuality });

    write(os->get(), buf.data(), buf.size());
    os->close();
}

Mesh loadMesh(const IStream::pointer &is)
{
    auto mesh(loadBinaryMesh(is->get()));
    is->close();
    return mesh;
}

void saveMesh(const OStream::pointer &os, const Mesh &mesh)
{
    writeBinaryMesh(os->get(), mesh);
    os->close();
}

} // namespace

struct TileSet::Factory
{
    static TileSet::pointer create(const Locator &locator
                                   , const CreateProperties &properties
                                   , CreateMode mode)
    {
        auto driver(Driver::create(locator, mode));
        return TileSet::pointer(new TileSet(driver, properties));
    }

    static TileSet::pointer open(const Locator &locator, OpenMode mode)
    {
        auto driver(Driver::open(locator, mode));
        return TileSet::pointer(new TileSet(driver));
    }

    static void clone(const TileSet::pointer &src
                      , const TileSet::pointer &dst)
    {
        dst->detail().clone(src->detail());
    }
};

/** This simple class allows access to the "detail" implementation of a tile
 * set.
 */
struct TileSet::Accessor
{
    static Detail& detail(TileSet &ts) { return ts.detail(); }
    static const Detail& detail(const TileSet &ts) { return ts.detail(); }
};

TileSet::pointer createTileSet(const Locator &locator
                               , const CreateProperties &properties
                               , CreateMode mode)
{
    return TileSet::Factory::create(locator, properties, mode);
}

TileSet::pointer openTileSet(const Locator &locator, OpenMode mode)
{
    return TileSet::Factory::open(locator, mode);
}

TileSet::pointer cloneTileSet(const Locator &locator, const Locator &srcLocator
                              , CreateMode mode)
{
    return cloneTileSet(locator, openTileSet(srcLocator, OpenMode::readOnly)
                        , mode);
}

TileSet::pointer cloneTileSet(const Locator &locator
                              , const TileSet::pointer &src
                              , CreateMode mode)
{
    auto dst(createTileSet(locator, src->getProperties(), mode));
    TileSet::Factory::clone(src, dst);
    return dst;
}

TileSet::pointer cloneTileSet(const TileSet::pointer &dst
                              , const TileSet::pointer &src
                              , CreateMode mode)
{
    // make sure dst is flushed
    dst->flush();
    if (!dst->empty() && (mode != CreateMode::overwrite)) {
        LOGTHROW(err2, TileSetNotEmpty)
            << "Tile set <" << dst->getProperties().id
            << "> is not empty.";
    }
    TileSet::Factory::clone(src, dst);
    return dst;
}

TileSet::Detail::Detail(const Driver::pointer &driver)
    : driver(driver), propertiesChanged(false)
    , metadataChanged(false)
    , tx(false)
{
    loadConfig();
    // load tile index only if foat is valid
    if (properties.foatSize) {
        loadTileIndex();
    } else {
        tileIndex = { properties.baseTileSize };
    }
}

TileSet::Detail::Detail(const Driver::pointer &driver
                        , const CreateProperties &properties)
    : driver(driver), propertiesChanged(false)
    , metadataChanged(false)
    , tx(false)
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

    // build initial properties
    auto &p(this->properties);

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

    // tile index must be properly initialized
    tileIndex = { p.baseTileSize };

    saveConfig();
}

TileSet::Detail::~Detail()
{
    // no exception thrown and not flushe? warn user!
    if (!std::uncaught_exception()
        && (metadataChanged || propertiesChanged))
    {
        LOG(warn3) << "Tile set is not flushed on destruction: "
            "data could be unusable.";
    }
}

void TileSet::Detail::loadConfig()
{
    // load json
    try {
        auto f(driver->input(File::config));
        Json::Reader reader;
        if (!reader.parse(*f, config)) {
            LOGTHROW(err2, FormatError)
                << "Unable to parse config: "
                << reader.getFormattedErrorMessages() << ".";
        }
        f->close();
    } catch (const std::exception &e) {
        LOGTHROW(err2, Error)
            << "Unable to read config: <" << e.what() << ">.";
    }

    Properties p;
    parse(p, config);
    properties = p;
    savedProperties = properties = p;
}

void TileSet::Detail::saveConfig()
{
    LOG(info1)
        << "Saving properties:\n"
        << utility::dump(properties, "    ");

    // save json
    try {
        driver->wannaWrite("save config");
        build(config, properties);
        auto f(driver->output(File::config));
        f->get().precision(15);
        Json::StyledStreamWriter().write(*f, config);
        f->close();
    } catch (const std::exception &e) {
        LOGTHROW(err2, Error)
            << "Unable to write config: <" << e.what() << ">.";
    }
    // done; remember saved properties and go on
    savedProperties = properties;
    propertiesChanged = false;
}

void TileSet::Detail::saveMetadata()
{
    driver->wannaWrite("save metadata");

    // create tile index (initialize parameters with existing one)
    TileIndex ti(properties.alignment, properties.baseTileSize
                 , extents, lodRange, &tileIndex);

    // create metatile index (initialize parameters with existing one)
    TileIndex mi(properties.alignment, properties.baseTileSize
                 , extents, {properties.foat.lod, lodRange.max}
                 , &ti);

    // purge out nonexistent leaves from metadata tree
    purgeMetadata();

    // well, dump metatiles now
    saveMetatiles(ti, mi);

    // save index
    try {
        auto f(driver->output(File::tileIndex));
        ti.save(*f);
        mi.save(*f);
        f->close();
    } catch (const std::exception &e) {
        LOGTHROW(err2, Error)
            << "Unable to write tile index: " << e.what() << ".";
    }

    // cool, we have new tile index
    tileIndex = ti;

    if (metadata.empty()) {
        // no tile, we should invalidate foat
        properties.foat = {};
        propertiesChanged = true;
        LOG(info2) << "Tile set <" << properties.id << ">: New foat is "
                   << properties.foat << ".";
    }

    // saved => no change
    metadataChanged = false;
}

void TileSet::Detail::loadTileIndex()
{
    try {
        tileIndex = { properties.baseTileSize };
        metaIndex = { properties.baseTileSize };
        auto f(driver->input(File::tileIndex));
        tileIndex.load(*f);
        metaIndex.load(*f);
        f->close();
    } catch (const std::exception &e) {
        LOGTHROW(err2, Error)
            << "Unable to read tile index: " << e.what() << ".";
    }

    // new extents
    lodRange = tileIndex.lodRange();
    extents = tileIndex.extents();
    LOG(info2) << "Loaded tile index: " << tileIndex;
}

MetaNode* TileSet::Detail::findMetaNode(const TileId &tileId)
    const
{
    auto fmetadata(metadata.find(tileId));
    if (fmetadata == metadata.end()) {
        // not found in memory -> load from disk
        return loadMetatile(tileId);
    }

    return &fmetadata->second;
}

MetaNode TileSet::Detail::setMetaNode(const TileId &tileId
                                      , const MetaNode& metanode)
{
    // this ensures that we have old metanode in memory
    findMetaNode(tileId);

    // insert new node or update existing
    auto res(metadata.insert(Metadata::value_type(tileId, metanode)));
    auto &newNode(res.first->second);
    if (!res.second) {
        newNode = metanode;
    }

    // update extents/lod-range; only when tile is valid
    if (valid(metanode)) {
        if (math::empty(extents)) {
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
    }

    // remember old foat
    auto oldFoat(properties.foat);

    // layout updated -> update zboxes up the tree
    updateTree(tileId, newNode);

    // if added tile was outside of old foat we have to generate tree from old
    // foat to new foat (which was generated above)
    if (!above(properties.baseTileSize, tileId, oldFoat)) {
        updateTree(oldFoat);
    }

    metadataChanged = true;

    // OK, return new node
    return newNode;
}

MetaNode&
TileSet::Detail::createVirtualMetaNode(const TileId &tileId)
{
    // this ensures that we have old metanode in memory
    auto *md(findMetaNode(tileId));
    if (!md) {
        // no node (real or virtual), create virtual node
        md = &(metadata.insert(Metadata::value_type(tileId, MetaNode()))
               .first->second);
        LOG(info2) << "(" << properties.id
                   << "): Created virtual tile " << tileId << ".";
    }

    metadataChanged = true;

    // ok
    return *md;
}

void TileSet::Detail::loadMetatileFromFile(const TileId &tileId) const
{
    auto f(driver->input(tileId, TileFile::meta));
    tilestorage::loadMetatile
        (*f, properties.baseTileSize, tileId
         , [this] (const TileId &tileId, const MetaNode &node
                   , std::uint8_t)
         {
             metadata.insert(Metadata::value_type(tileId, node));
         });
    f->close();

    loadedMetatiles.insert(tileId);
}

MetaNode* TileSet::Detail::loadMetatile(const TileId &tileId)
    const
{
    // if tile is not marked in the index it cannot be found on the disk
    if (in(lodRange, tileId) && !tileIndex.exists(tileId)) {
        return nullptr;
    }

    // no tile (or metatile) cannot be in an lod below lowest level in the tile
    // set)
    if (tileId.lod > lodRange.max) {
        return nullptr;
    }

    // sanity check
    if (!savedProperties.foatSize) {
        // no data on disk
        return nullptr;
    }

    if (!above(savedProperties.baseTileSize, tileId, savedProperties.foat)) {
        // foat is not above tile -> cannot be present on disk
        return nullptr;
    }

    auto metaId(tileId);
    while ((metaId != savedProperties.foat)
           && !isMetatile(savedProperties.metaLevels, metaId))
    {
        metaId = tilestorage::parent(savedProperties.alignment
                                     , savedProperties.baseTileSize
                                     , metaId);
    }

    if (!metaIndex.exists(metaId)) {
        return nullptr;
    }

    if (loadedMetatiles.find(metaId) != loadedMetatiles.end()) {
        // this metatile already loaded
        return nullptr;
    }

    LOG(info2) << "(" << properties.id << "): Found metatile "
               << metaId << " for tile " << tileId << ".";

    loadMetatileFromFile(metaId);

    // now, we can execute lookup again
    auto fmetadata(metadata.find(tileId));
    if (fmetadata != metadata.end()) {
        LOG(info1) << "(" << properties.id << "): Meta node for "
            "tile " << tileId << " loaded from disk.";
        return &fmetadata->second;
    }

    return nullptr;
}

void TileSet::Detail::setMetadata(const TileId &tileId
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

void TileSet::Detail::check(const TileId &tileId) const
{
    auto ts(tileSize(properties, tileId.lod));

    auto aligned(fromAlignment(properties, tileId));
    if ((aligned.easting % ts) || (aligned.northing % ts)) {
        LOGTHROW(err2, NoSuchTile)
            << "Misaligned tile at " << tileId << " cannot exist.";
    }
}

void TileSet::Detail::updateTree(const TileId &tileId)
{
    if (auto *node = findMetaNode(tileId)) {
        updateTree(tileId, *node);
    }
}

void TileSet::Detail::updateTree(const TileId &tileId
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
            LOG(info2) << "Tile set <" << properties.id
                       << ">: New foat is " << properties.foat << ".";
        }
        return;
    }

    auto parentId(parent(tileId));
    if (auto *parentNode = findMetaNode(parentId)) {
        updateTree(parentId, *parentNode);
    } else {
        // there is no parent present in the tree; process freshly generated
        // parent
        updateTree(parentId, createVirtualMetaNode(parentId));
    }
}

bool TileSet::Detail::isFoat(const TileId &tileId) const
{
    if (extents.ll(0) < tileId.easting) { return false; }
    if (extents.ll(1) < tileId.northing) { return false; }

    auto ts(tileSize(properties.baseTileSize, tileId.lod));

    if (extents.ur(0) > (tileId.easting + ts)) { return false; }
    if (extents.ur(1) > (tileId.northing + ts)) { return false; }

    return true;
}

void TileSet::Detail::flush()
{
    if (driver->readOnly()) { return; }
    LOG(info3) << "Tile set <" << properties.id << ">: flushing.";

    // force metadata save
    if (metadataChanged) {
        saveMetadata();
    }

    // force config save
    if (propertiesChanged) {
        saveConfig();
    }
    LOG(info3) << "Tile set <" << properties.id << ">: flushed.";
}

void TileSet::Detail::purgeMetadata()
{
    struct Crawler {
        Crawler(TileSet::Detail &detail) : detail(detail) {}

        /** Processes given tile and returns whether tile was kept in the tree
         */
        bool run(const TileId &tileId) {
            const auto *node(detail.findMetaNode(tileId));
            if (!node) { return false; }

            LOG(info1) << "(purge): " << tileId;

            int hasChild(0);

            // process children
            for (auto childId
                     : children(detail.properties.baseTileSize, tileId))
            {
                hasChild += run(childId);
            }

            if (!hasChild && !node->exists()) {
                // no children and no content -> kill
                detail.metadata.erase(tileId);
                detail.metadataChanged = true;
                return false;
            }

            // keep this node
            return true;
        }

        TileSet::Detail &detail;
    };

    Crawler(*this).run(properties.foat);
}

void TileSet::Detail::saveMetatiles(TileIndex &tileIndex, TileIndex &metaIndex)
    const
{
    struct Saver : MetaNodeSaver {
        const TileSet::Detail &detail;
        TileIndex &tileIndex;
        TileIndex &metaIndex;
        Saver(const TileSet::Detail &detail
              , TileIndex &tileIndex, TileIndex &metaIndex)
            : detail(detail), tileIndex(tileIndex), metaIndex(metaIndex)
        {}

        virtual void saveTile(const TileId &metaId
                              , const MetaTileSaver &saver)
            const UTILITY_OVERRIDE
        {
            metaIndex.set(metaId);
            auto f(detail.driver->output(metaId, TileFile::meta));
            saver(*f);
            f->close();
        }

        virtual const MetaNode* getNode(const TileId &tileId)
            const UTILITY_OVERRIDE
        {
            auto *node(detail.findMetaNode(tileId));
            tileIndex.set(tileId, node && node->exists());
            return node;
        }
    };

    tilestorage::saveMetatile(properties.baseTileSize, properties.foat
                              , properties.metaLevels
                              , Saver(*this, tileIndex, metaIndex));
}

Tile TileSet::Detail::getTile(const TileId &tileId) const
{
    check(tileId);

    auto md(findMetaNode(tileId));
    if (!md || !md->exists()) {
        LOGTHROW(err2, NoSuchTile)
            << "There is no tile at " << tileId << ".";
    }

    // TODO: pass fake path to loaders
    return {
        loadMesh(driver->input(tileId, TileFile::mesh))
        , loadAtlas(driver->input(tileId, TileFile::atlas))
        , *md
    };
}

boost::optional<Tile> TileSet::Detail::getTile(const TileId &tileId
                                               , std::nothrow_t)
    const
{
    check(tileId);

    auto md(findMetaNode(tileId));
    if (!md || !md->exists()) { return boost::none; }

    return boost::optional<Tile>
        (boost::in_place
         (loadMesh(driver->input(tileId, TileFile::mesh))
          , loadAtlas(driver->input(tileId, TileFile::atlas))
          , *md));
}

MetaNode TileSet::Detail::setTile(const TileId &tileId, const Mesh &mesh
                                  , const Atlas &atlas
                                  , const TileMetadata *metadata)
{
    driver->wannaWrite("set tile");

    check(tileId);

    LOG(info1) << "Setting content of tile " << tileId << ".";

    // create new metadata
    MetaNode metanode;

    // copy extra metadata
    if (metadata) {
        static_cast<TileMetadata&>(metanode) = *metadata;
    }

    // calculate dependent metadata
    metanode.calcParams(mesh, { atlas.cols, atlas.rows });

    if (metanode.exists()) {
        // save data only if valid
        saveMesh(driver->output(tileId, TileFile::mesh), mesh);
        saveAtlas(driver->output(tileId, TileFile::atlas)
                  , atlas, properties.textureQuality);
    } else {
        LOG(info1) << "Tile " << tileId << " has no content.";

        // remove mesh and atlas from storage
        driver->remove(tileId, TileFile::mesh);
        driver->remove(tileId, TileFile::atlas);
    }

    // remember new metanode (can lead to removal of node)
    return setMetaNode(tileId, metanode);
}

MetaNode TileSet::Detail::removeTile(const TileId &tileId)
{
    driver->wannaWrite("remove tile");

    check(tileId);

    LOG(info1) << "Removing tile " << tileId << ".";

    // create new metadata
    MetaNode metanode;

    // remove mesh and atlas from storage
    driver->remove(tileId, TileFile::mesh);
    driver->remove(tileId, TileFile::atlas);

    // remember empty metanode (leads to removal of node)
    return setMetaNode(tileId, metanode);
}

void TileSet::Detail::begin()
{
    LOG(info3)
        << "Tile set <" << properties.id << ">: Opening transaction.";

    driver->wannaWrite("begin transaction");
    if (tx) {
        LOGTHROW(err2, PendingTransaction)
            << "Transaction already in progress.";
    }

    driver->begin();

    tx = true;
}

void TileSet::Detail::commit()
{
    LOG(info3)
        << "Tile set <" << properties.id << ">: Commiting transaction.";
    driver->wannaWrite("commit transaction");

    if (!tx) {
        LOGTHROW(err2, PendingTransaction)
            << "There is no active transaction to commit.";

    }
    // forced flush
    flush();

    driver->commit();

    tx = false;
}

void TileSet::Detail::rollback()
{
    LOG(info3)
        << "Tile set <" << properties.id << ">: Rolling back transaction.";
    if (!tx) {
        LOGTHROW(err2, PendingTransaction)
            << "There is no active transaction to roll back.";

    }

    // TODO: what to do if anything throws?

    driver->rollback();

    // re-read backing store state
    loadConfig();

    // destroy tile index
    tileIndex = { savedProperties.baseTileSize };
    metadata = {};
    loadedMetatiles = {};
    metadataChanged = false;

    if (savedProperties.foatSize) {
        // load tile index only if foat is valid
        loadTileIndex();
    }

    // no pending tx
    tx = false;
}

// tileSet itself

TileSet::TileSet(const Driver::pointer &driver)
    : detail_(new Detail(driver))
{
}

TileSet::TileSet(const Driver::pointer &driver
                 , const CreateProperties &properties)
    : detail_(new Detail(driver, properties))
{
}

TileSet::~TileSet()
{
}

void TileSet::flush()
{
    detail().flush();
}

Tile TileSet::getTile(const TileId &tileId) const
{
    return detail().getTile(tileId);
}

void TileSet::setTile(const TileId &tileId, const Mesh &mesh
                      , const Atlas &atlas, const TileMetadata *metadata)
{
    detail().setTile(tileId, mesh, atlas, metadata);
}

void TileSet::removeTile(const TileId &tileId)
{
    detail().removeTile(tileId);
}

void TileSet::setMetadata(const TileId &tileId, const TileMetadata &metadata)
{
    detail().driver->wannaWrite("set tile metadata");

    detail().check(tileId);

    detail().setMetadata(tileId, metadata);
}

bool TileSet::tileExists(const TileId &tileId) const
{
    // have look to in-memory data
    const auto &metadata(detail().metadata);
    auto fmetadata(metadata.find(tileId));
    if (fmetadata != metadata.end()) {
        return fmetadata->second.exists();
    }

    // try tileIndex
    return detail().tileIndex.exists(tileId);
}

Properties TileSet::getProperties() const
{
    return detail().properties;
}

Properties TileSet::setProperties(const SettableProperties &properties
                                  , SettableProperties::MaskType mask)
{
    detail().driver->wannaWrite("set properties");

    // merge in new properties
    if (detail().properties.merge(properties, mask)) {
        detail().propertiesChanged = true;
    }
    return detail().properties;
}

void TileSet::begin()
{
    detail().begin();
}

void TileSet::commit()
{
    detail().commit();
}

void TileSet::rollback()
{
    detail().rollback();
}

namespace {

LodRange range(const TileSet::list &sets)
{
    if (sets.empty()) {
        // empty set
        return { 0, -1 };
    }

    // initialize with invalid range
    LodRange r(0, -1);
    for (const auto &set : sets) {
        r = unite(r, TileSet::Accessor::detail(*set).lodRange);
    }
    return r;
}

typedef std::vector<const TileIndex*> TileIndices;

inline void tileIndicesImpl(TileIndices &indices, const TileSet::list &set)
{
    // get all tile indices
    for (const auto &ts : set) {
        auto &detail(TileSet::Accessor::detail(*ts));
        // we need fresh index
        detail.flush();
        indices.push_back(&detail.tileIndex);
    }
}

template <typename ...Args>
inline void tileIndicesImpl(TileIndices &indices, const TileSet::list &set
                            , Args &&...rest)
{
    tileIndicesImpl(indices, set);
    tileIndicesImpl(indices, std::forward<Args>(rest)...);
}

template <typename ...Args>
inline TileIndices tileIndices(Args &&...args)
{
    TileIndices indices;
    tileIndicesImpl(indices, std::forward<Args>(args)...);
    return indices;
}

inline void dump(const boost::filesystem::path &dir, const TileSet::list &set)
{
    int i(0);
    for (const auto &s : set) {
        auto &detail(TileSet::Accessor::detail(*s));
        // we need fresh index
        detail.flush();
        LOG(info2) << "Dumping <" << s->getProperties().id << ">.";

        auto path(dir / str(boost::format("%03d") % i));
        dumpAsImages(path, detail.tileIndex);
        ++i;
    }
}

} // namespace

#ifdef VADSTENA_LIBS_DUMP_TILEINDEX
#    define DUMP_TILEINDEX(name, index) do {        \
            dumpAsImages(name, index);              \
            index.save(fs::path(name) / "raw.bin"); \
        } while (false)
#else
#    define DUMP_TILEINDEX(name, index) do {} while (false)
#endif // VADSTENA_LIBS_DUMP_TILEINDEX

void TileSet::mergeIn(const list &kept, const list &update)
{
    LOG(info3) << "(merge-in) Calculating generate set.";

#ifdef VADSTENA_LIBS_DUMP_TILEINDEX
    dump("debug/update", update);
#endif // VADSTENA_LIBS_DUMP_TILEINDEX

    const auto &alignment(detail().properties.alignment);

    // calculate storage update (we need to know lod range of kept sets to
    // ensure all indices cover same lod range)
    auto tsUpdate(unite(alignment, tileIndices(update), range(kept)));
    if (math::empty(tsUpdate.extents())) {
        LOG(warn3) << "(merge-in) Nothing to merge in. Bailing out.";
        return;
    }

    DUMP_TILEINDEX("debug/tsUpdate", tsUpdate);
    tsUpdate.growDown();
    DUMP_TILEINDEX("debug/tsUpdate-gd", tsUpdate);

    // calculate storage post state
    auto tsPost(unite(alignment, tileIndices(update, kept), tsUpdate));
    DUMP_TILEINDEX("debug/tsPost", tsPost);
    tsPost.growUp();
    DUMP_TILEINDEX("debug/tsPost-gu", tsPost);

    // calculate storage pre state
    auto tsPre(unite(alignment, tileIndices(kept), tsUpdate));
    DUMP_TILEINDEX("debug/tsPre", tsPre);
    tsPre.growUp().invert();
    DUMP_TILEINDEX("debug/tsPre-gu-inv", tsPre);

    LOG(info2) << "(merge-in) down(tsUpdate): " << tsUpdate;
    LOG(info2) << "(merge-in) up(tsPost): " << tsPost;
    LOG(info2) << "(merge-in) inv(up(tsPre)): " << tsPre;

    auto generate(intersect(alignment, tsPost
                            , unite(alignment, tsUpdate, tsPre)));
    DUMP_TILEINDEX("debug/generate", generate);

    LOG(info2) << "(merge-in) generate: " << generate;

    if (generate.empty()) {
        LOG(warn3) << "(merge-in) Nothing to generate. Bailing out.";
        return;
    }

    list all(kept.begin(), kept.end());
    all.insert(all.end(), update.begin(), update.end());

    LOG(info3) << "(merge-in) Generate set calculated.";

    auto lod(generate.minLod());
    auto s(generate.rasterSize(lod));
    for (long j(0); j < s.height; ++j) {
        for (long i(0); i < s.width; ++i) {
            LOG(info2) << "(merge-in) Processing subtree "
                       << lod << "/(" << i << ", " << j << ").";
            detail().mergeSubtree(generate, nullptr, {lod, i, j}, all);
        }
    }

    LOG(info3) << "(merge-in) Tile sets merged in.";

    // center default position if not inside tileset
    detail().fixDefaultPosition(all);
}

void TileSet::mergeOut(const list &kept, const list &update)
{
    LOG(info3) << "(merge-out) Calculating generate and remove sets.";

#ifdef VADSTENA_LIBS_DUMP_TILEINDEX
    dump("debug/update", update);
#endif // VADSTENA_LIBS_DUMP_TILEINDEX

    const auto &alignment(detail().properties.alignment);

    // calculate storage update (we need to know lod range of kept sets to
    // ensure all indices cover same lod range)
    auto tsUpdate(unite(alignment, tileIndices(update), range(kept)));
    if (math::empty(tsUpdate.extents())) {
        LOG(warn3) << "(merge-out) Nothing to merge in. Bailing out.";
        return;
    }

    DUMP_TILEINDEX("debug/tsUpdate", tsUpdate);
    tsUpdate.growDown();
    DUMP_TILEINDEX("debug/tsUpdate-gd", tsUpdate);

    // calculate storage post state
    auto tsPost(unite(alignment, tileIndices(kept), tsUpdate));
    DUMP_TILEINDEX("debug/tsPost", tsPost);
    tsPost.growUp();
    DUMP_TILEINDEX("debug/tsPost-gu", tsPost);

    // calculate storage pre state
    auto tsPre(unite(alignment, tileIndices(update, kept), tsUpdate));
    DUMP_TILEINDEX("debug/tsPre", tsPre);
    tsPre.growUp();
    DUMP_TILEINDEX("debug/tsPre-gu", tsPre);

    // create remove set
    auto remove(difference(alignment, tsPre, tsPost));
    DUMP_TILEINDEX("debug/remove", remove);

    tsPre.invert();
    DUMP_TILEINDEX("debug/tsPre-gu-inv", tsPre);

    LOG(info2) << "(merge-out) down(tsUpdate): " << tsUpdate;
    LOG(info2) << "(merge-out) up(tsPost): " << tsPost;
    LOG(info2) << "(merge-out) inv(up(tsPre)): " << tsPre;

    auto generate(intersect(alignment, tsPost
                            , unite(alignment, tsUpdate, tsPre)));
    DUMP_TILEINDEX("debug/generate", generate);

    LOG(info2) << "(merge-out) generate: " << generate;
    LOG(info2) << "(merge-out) remove: " << remove;


    list all(kept.begin(), kept.end());
    all.insert(all.end(), update.begin(), update.end());

    LOG(info3) << "(merge-in) Generate and remove sets calculated.";

    auto lod(generate.minLod());
    auto s(generate.rasterSize(lod));
    for (long j(0); j < s.height; ++j) {
        for (long i(0); i < s.width; ++i) {
            LOG(info2) << "(merge-out) Processing subtree "
                       << lod << "/(" << i << ", " << j << ").";
            detail().mergeSubtree(generate, &remove, {lod, i, j}, all);
        }
    }

    LOG(info3) << "(merge-in) Tile sets merged out.";

    // center default position if not inside tileset
    detail().fixDefaultPosition(all);
}

Tile TileSet::Detail::generateTile(const TileId &tileId
                                   , const TileSet::list &src
                                   , const Tile &parentTile
                                   , int quadrant)
{
    auto ts(tileSize(properties.baseTileSize, tileId.lod));

    // Fetch tiles from other tiles.
    Tile::list tiles;
    for (const auto &ts : src) {
        if (auto t = ts->detail().getTile(tileId, std::nothrow)) {
            tiles.push_back(*t);
        }
    }

    // optimization
    if (quadrant < 0) {
        // no parent data
        if (tiles.empty()) {
            // no data
            return {};
        } else if ((tiles.size() == 1)) {
            // just one single tile without any fallback
            auto tile(tiles.front());
            tile.metanode
                = setTile(tileId, tile.mesh, tile.atlas, &tile.metanode);
            return tile;
        }

        // more tiles => must merge
    }

    // we have to merge tiles
    auto tile(merge(ts, tiles, parentTile, quadrant));
    tile.metanode
        = setTile(tileId, tile.mesh, tile.atlas, &tile.metanode);
    return tile;
}

void TileSet::Detail::mergeSubtree(const TileIndex &generate
                                   , const TileIndex *remove
                                   , const Index &index
                                   , const TileSet::list &src
                                   , const Tile &parentTile
                                   , int quadrant
                                   , bool parentGenerated)
{
    // tile generated in this run (empty and invalid by default)
    Tile tile;

    // should this tile be generated?
    auto g(generate.exists(index));
    auto r(remove && remove->exists(index));
    if (g) {
        auto tileId(generate.tileId(index));
        LOG(info2) << "(merge-in) Processing tile "
                   << index << ", " << tileId << ".";

        bool thisGenerated(false);
        if (!parentGenerated) {
            if (auto t = getTile(parent(tileId), std::nothrow)) {
                // no parent was generated and we have sucessfully loaded parent
                // tile from existing content as a fallback tile!

                LOG(info2) << "Parent tile loaded.";
                quadrant = child(index);
                tile = generateTile(tileId, src, *t, quadrant);
                thisGenerated = true;
            }
        }

        if (!thisGenerated) {
            // regular generation
            tile = generateTile(tileId, src, parentTile, quadrant);
        }
    } else if (r) {
        auto tileId(generate.tileId(index));
        LOG(info2) << "(merge-out) Processing tile "
                   << index << ", " << tileId << ".";
        removeTile(tileId);
    } else if (parentGenerated && !remove) {
        // parent was generated but this tile will not be generated and no
        // tiles are about to be removed => reached bottom of this subreee
        return;
    }

    // can we go down?
    if (index.lod >= generate.maxLod()) {
        // no way down
        return;
    }

    // OK, process children

    // if tile doesn't exist generated quadrants are not valid -> not included
    // in merge operation
    quadrant = valid(tile) ? 0 : MERGE_NO_FALLBACK_TILE;
    for (const auto &child : children(index)) {
        mergeSubtree(generate, remove, child, src, tile, quadrant, g);
        if (quadrant != MERGE_NO_FALLBACK_TILE) {
            ++quadrant;
        }
    }
}

void TileSet::Detail::fixDefaultPosition(const list &tileSets)
{
    double maxHeight(400);
    for (const auto &ts : tileSets) {
        maxHeight = std::max(ts->getProperties().defaultPosition(2)
                             , maxHeight);
    }

    if (!inside(extents, properties.defaultPosition)) {
        if (tileSets.empty()) {
            auto c(center(extents));
            properties.defaultPosition(0) = c(0);
            properties.defaultPosition(1) = c(1);
            properties.defaultPosition(2) = maxHeight;
            propertiesChanged = true;
        } else {
            properties.defaultPosition
                = tileSets.front()->getProperties().defaultPosition;
            properties.defaultPosition(2) = maxHeight;
        }
    }
}

namespace {
    void copyFile(const IStream::pointer &in
                  , const OStream::pointer &out)
    {
        out->get() << in->get().rdbuf();
        in->close();
        out->close();
    }
}

void TileSet::Detail::clone(const Detail &src)
{
    const auto &sd(*src.driver);
    auto &dd(*driver);

    // copy single files
    for (auto type : { File::config, File::tileIndex }) {
        copyFile(sd.input(type), dd.output(type));
    }

    // copy tiles
    auto traverser(src.tileIndex.traverser());
    while (auto tile = traverser.next()) {
        if (!tile.value) { continue; }
        for (auto type : { TileFile::mesh, TileFile::atlas }) {
            copyFile(sd.input(tile.id, type), dd.output(tile.id, type));
        }
    }

    // copy metatiles
    traverser = src.metaIndex.traverser();
    while (auto tile = traverser.next()) {
        if (!tile.value) { continue; }
        copyFile(sd.input(tile.id, TileFile::meta)
                 , dd.output(tile.id, TileFile::meta));
    }
}

bool TileSet::empty() const
{
    // no foat -> no tile
    return !detail().properties.foatSize;
}

TileSet::AdvancedApi TileSet::advancedApi()
{
    return TileSet::AdvancedApi(shared_from_this());
}

Traverser TileSet::AdvancedApi::tileTraverser() const
{
    return Traverser(&tileSet_->detail().tileIndex);
}

Traverser TileSet::AdvancedApi::metaTraverser() const
{
    return Traverser(&tileSet_->detail().metaIndex);
}

OStream::pointer TileSet::AdvancedApi::output(File type)
{
    return tileSet_->detail().driver->output(type);
}

IStream::pointer TileSet::AdvancedApi::input(File type) const
{
    return tileSet_->detail().driver->input(type);
}

OStream::pointer TileSet::AdvancedApi::output(const TileId tileId
                                              , TileFile type)
{
    return tileSet_->detail().driver->output(tileId, type);
}

IStream::pointer TileSet::AdvancedApi::input(const TileId tileId
                                             , TileFile type) const
{
    return tileSet_->detail().driver->input(tileId, type);
}

} } // namespace vadstena::tilestorage
