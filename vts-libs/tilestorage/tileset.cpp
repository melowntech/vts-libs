#include <cstdlib>
#include <queue>

#include <boost/format.hpp>
#include <boost/utility/in_place_factory.hpp>

#include <opencv2/highgui/highgui.hpp>

#include "dbglog/dbglog.hpp"
#include "utility/binaryio.hpp"
#include "utility/progress.hpp"

#include "geo/srsdef.hpp"

#include "../binmesh.hpp"
#include "../tilestorage.hpp"
#include "./config.hpp"
#include "./tileindex.hpp"
#include "./io.hpp"
#include "./tileop.hpp"
#include "./tileset-detail.hpp"
#include "./tileset-advanced.hpp"
#include "./metatile.hpp"
#include "./merge.hpp"
#include "./tileset/dump.hpp"

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
                                   , CreateMode mode
                                   , bool cloned = false)
    {
        auto driver(Driver::create
                    (locator, mode, { properties.staticProperties, cloned }));
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

    static void clone(const TileSet::pointer &src
                      , const TileSet::pointer &dst
                      , const boost::optional<Extents> &extents
                      , const boost::optional<LodRange> &lodRange)
    {
        if (!extents && !lodRange) {
            return clone(src, dst);
        }

        dst->detail().clone(src->detail(), extents, lodRange);
    }
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
                              , const CloneOptions &options)
{
    return cloneTileSet(locator, openTileSet(srcLocator, OpenMode::readOnly)
                        , options);
}

TileSet::pointer cloneTileSet(const Locator &locator
                              , const TileSet::pointer &src
                              , const CloneOptions &options)
{
    // merge existing properties with new properties
    CreateProperties properties(src->getProperties());
    properties.staticProperties.merge(options.staticProperties, options.mask);

    // create tileset and clone
    auto dst(TileSet::Factory::create
             (locator, properties, options.createMode, true));
    TileSet::Factory::clone(src, dst, options.extents, options.lodRange);
    return dst;
}

TileSet::pointer cloneTileSet(const TileSet::pointer &dst
                              , const TileSet::pointer &src
                              , const CloneOptions &options)
{
    // make sure dst is flushed
    dst->flush();
    if (!dst->empty() && (options.createMode != CreateMode::overwrite)) {
        LOGTHROW(err2, TileSetNotEmpty)
            << "Tile set <" << dst->getProperties().id
            << "> is not empty.";
    }
    TileSet::Factory::clone(src, dst, options.extents, options.lodRange);
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
    static_cast<StaticProperties&>(p)
        .merge(properties.staticProperties, properties.mask);
    static_cast<SettableProperties&>(p)
        .merge(properties.settableProperties, properties.mask);

    // force save of properties
    p.driver = driver->properties();

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
        LOG(warn3)
            << "Tile set <" << properties.id
            << "> is not flushed on destruction: data could be unusable.";
    }
}

void TileSet::Detail::setFoat(const TileId &tileId)
{
    properties.foat = tileId;
    properties.foatSize = tileSize(properties, tileId.lod);
    propertiesChanged = true;
}

void TileSet::Detail::resetFoat()
{
    properties.foat = {};
    properties.foatSize = 0;
    propertiesChanged = true;
}

void TileSet::Detail::loadConfig()
{
    try {
        // load config
        auto f(driver->input(File::config));
        const auto p(tilestorage::loadConfig(*f));
        f->close();

        // set
        savedProperties = properties = p;
    } catch (const std::exception &e) {
        LOGTHROW(err2, Error)
            << "Unable to read config: <" << e.what() << ">.";
    }
}

void TileSet::Detail::saveConfig()
{
    // save json
    try {
        driver->wannaWrite("save config");
        auto f(driver->output(File::config));
        tilestorage::saveConfig(*f, properties);
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

    // purge out nonexistent leaves from metadata tree (recalculates extents)
    purgeMetadata();

    // create tile index
    TileIndex ti(properties.alignment, properties.baseTileSize
                 , extents, lodRange);
    LOG(info1) << "New tile index:\n" << ti;

    // create metatile index
    TileIndex mi(properties.alignment, properties.baseTileSize
                 , extents, {properties.foat.lod, lodRange.max});
    LOG(info1) << "New metatile index:\n" << mi;

    // well, dump metatiles now
    saveMetatiles(ti, mi);

    dropRemovedMetatiles(metaIndex, mi);

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

    // cool, we have new tile/meta index
    tileIndex = ti;
    metaIndex = mi;

    if (metadata.empty()) {
        // no tile, we should invalidate foat
        resetFoat();
        LOG(info1) << "Tile set <" << properties.id << ">: New foat is "
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
    auto old(findMetaNode(tileId));
    
    // update existing node
    if (old) {
        *old = metanode;
    }
    
    // invalid node = node removal -> no metadata update
    if (!valid(metanode)) {
        return metanode;
    }

    // insert new node
    if (!old) {
        metadata.insert(Metadata::value_type(tileId, metanode));
    }
    
    // now there surely is one
    auto newNode(*findMetaNode(tileId));

    // update extents/lod-range
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

void TileSet::Detail
::loadMetatileFromFile(Metadata &metadata, const TileId &tileId
                       , const MetaNodeNotify &notify)
    const
{
    auto f(driver->input(tileId, TileFile::meta));
    tilestorage::loadMetatile
        (*f, properties.baseTileSize, tileId
         , [&metadata]
         (const TileId &tileId, const MetaNode &node, std::uint8_t)
         {
             metadata.insert(Metadata::value_type(tileId, node));
         }
         , notify);
    f->close();
}

MetaNode* TileSet::Detail::loadMetatile(const TileId &tileId)
    const
{
    /* following condition is wrong for merged data, where
     * two or more tilesets with no intersecting area are merged and one tileset
     * has different lodRange.min than the other one. Virtual tiles can exist even
     * in the lodRange
     */ 
    // if tile is not marked in the index it cannot be found on the disk
    /*
    if (in(lodRange, tileId) && !tileIndex.exists(tileId)) {
        return nullptr;
    }
    */ 

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

    LOG(info1) << "(" << properties.id << "): Found metatile "
               << metaId << " for tile " << tileId << ".";

    loadMetatileFromFile(metadata, metaId);
    loadedMetatiles.insert(metaId);

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

    metadataChanged = true;
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

void TileSet::Detail::checkTx(const std::string &action) const
{
    if (!tx) {
        LOGTHROW(err2, PendingTransaction)
            << "Cannot " << action << ": no transaction open.";
    }
}

void TileSet::Detail::updateTreeMetadata(const TileId &tileId)
{
    if (auto *node = findMetaNode(tileId)) {
        updateTreeMetadata(tileId, *node);
    }
}

void TileSet::Detail::updateTreeMetadata(const TileId &tileId
                                 , MetaNode &metanode)
{
    // process all 4 children
    for (const auto &childId : children(properties.baseTileSize, tileId)) {
        if (auto *node = findMetaNode(childId)) {
            metanode.gsd = std::min(metanode.gsd, node->gsd);
        }
    }

    auto parentId(parent(tileId));
    if (auto *parentNode = findMetaNode(parentId)) {
        updateTreeMetadata(parentId, *parentNode);
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
    float minGsd = std::numeric_limits<float>::max();
    bool minGsdSet = false;

    // process all 4 children
    for (const auto &childId : children(properties.baseTileSize, tileId)) {
        if (auto *node = findMetaNode(childId)) {
            metanode.zmin = std::min(metanode.zmin, node->zmin);
            metanode.zmax = std::max(metanode.zmax, node->zmax);
            minGsd = std::min(minGsd, node->gsd);
            minGsdSet = true;
        }
    }

    if(minGsdSet){
        metanode.gsd = minGsd;
    }
    // this tile is (current) foat -> no parent can ever exist (until real tile
    // is added)
    if (isFoat(tileId)) {
        // we reached foat tile => way up
        if (tileId != properties.foat) {
            setFoat(tileId);
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

    // flush driver
    driver->flush();
    LOG(info3) << "Tile set <" << properties.id << ">: flushed.";
}


void TileSet::Detail::removeOverFoat()
{
    // this is purgeMetadata helper function
    for (;;) {
        const auto tileId(properties.foat);

        const auto *node(findMetaNode(tileId));
        if (!node || node->exists()) {
            // there is no such node (eh?) or it has valid tile
            return;
        }

        boost::optional<TileId> foatChild;
        // process children
        for (auto childId : children(properties.baseTileSize, tileId)) {
            if (findMetaNode(childId)) {
                if (foatChild) {
                    // more than one child -> cannot trim foat
                    return;
                }
                // we have foat child!
                foatChild = childId;
            }
        }

        if (!foatChild) {
            // current foat has more children -> done
            return;
        }

        // single foat child -> we can remove foat
        driver->remove(tileId, TileFile::meta);
        metadata.erase(tileId);
        metadataChanged = true;

        // remember new foat
        setFoat(*foatChild);
        LOG(info2) << "Decreased foat to " << properties.foat;

        // try next level
    }
}

void TileSet::Detail::purgeMetadata()
{
    struct Crawler {
        Crawler(TileSet::Detail &detail, Extents &extents
                , LodRange &lodRange)
            : detail(detail), extents(extents), lodRange(lodRange)
        {}

        /** Processes given tile and returns whether tile was kept in the tree
         */
        bool purge(const TileId &tileId) {
            const auto *node(detail.findMetaNode(tileId));
            if (!node) { return false; }

            LOG(info1) << "(purge): " << tileId;

            int hasChild(0);
            // process children
            for (auto childId
                     : children(detail.properties.baseTileSize, tileId))
            {
                hasChild += purge(childId);
            }

            if (!hasChild && !node->exists()) {
                // no children and no content -> kill
                detail.driver->remove(tileId, TileFile::meta);
                detail.metadata.erase(tileId);
                detail.metadataChanged = true;
                return false;
            } else if (node->exists()) {
                // update extents
                extents = unite(extents, tileExtents
                                (detail.properties.baseTileSize, tileId));

                if (lodRange.empty()) {
                    lodRange.min = lodRange.max = tileId.lod;
                } else if (tileId.lod < lodRange.min) {
                    lodRange.min = tileId.lod;
                } else if (tileId.lod > lodRange.max) {
                    lodRange.max = tileId.lod;
                }
            }

            // keep this node
            return true;
        }

        TileSet::Detail &detail;
        Extents &extents;
        LodRange &lodRange;
    };

    Extents newExtents(math::InvalidExtents{});
    LodRange newLodRange(0, -1);

    if (!Crawler(*this, newExtents, newLodRange).purge(properties.foat)) {
        // FOAT has been removed! Empty storage!
        resetFoat();
        LOG(info2) << "Tile set <" << properties.id << ">: New foat is "
                   << properties.foat << ".";
        return;
    }

    // update extents
    if ((extents.ll != newExtents.ll) || (extents.ur != newExtents.ur)) {
        LOG(info3) << "Changing extents from " << extents
                   << " to " << newExtents << ".";
        extents = newExtents;
    }

    if ((lodRange.min != newLodRange.min)
        || (lodRange.max != newLodRange.max))
    {
        LOG(info3) << "Changing lod range from " << lodRange
                   << " to " << newLodRange << ".";
        lodRange = newLodRange;
    }

    // remove over FOAT
    removeOverFoat();
}

void TileSet::Detail::saveMetatiles(TileIndex &tileIndex, TileIndex &metaIndex)
    const
{
    if (!properties.foatSize) {
        // no data
        return;
    }

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
                                  , const TileMetadata *metadata
                                  , const boost::optional<double> &pixelSize)
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
    metanode.calcParams(mesh, { atlas.cols, atlas.rows }, pixelSize);

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

void TileSet::Detail::begin(utility::Runnable *runnable)
{
    LOG(info3)
        << "Tile set <" << properties.id << ">: Opening transaction.";

    driver->wannaWrite("begin transaction");
    if (tx) {
        LOGTHROW(err2, PendingTransaction)
            << "Transaction already in progress.";
    }

    driver->begin(runnable);

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

    LOG(info3) << "Tile set <" << properties.id << ">: commit started.";
    driver->commit();
    LOG(info3) << "Tile set <" << properties.id << ">: commit finished.";

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
    propertiesChanged = false;

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
{}

TileSet::~TileSet()
{
}

void TileSet::flush()
{
    detail().checkValidity();
    detail().flush();
}

Tile TileSet::getTile(const TileId &tileId) const
{
    detail().checkValidity();
    return detail().getTile(tileId);
}

void TileSet::setTile(const TileId &tileId, const Mesh &mesh
                      , const Atlas &atlas, const TileMetadata *metadata
                      , const boost::optional<double> &pixelSize)
{
    detail().checkValidity();
    detail().setTile(tileId, mesh, atlas, metadata, pixelSize);
}

void TileSet::removeTile(const TileId &tileId)
{
    detail().checkValidity();
    detail().removeTile(tileId);
}

void TileSet::setMetadata(const TileId &tileId, const TileMetadata &metadata)
{
    detail().checkValidity();

    detail().driver->wannaWrite("set tile metadata");

    detail().check(tileId);

    detail().setMetadata(tileId, metadata);
}

bool TileSet::tileExists(const TileId &tileId) const
{
    detail().checkValidity();

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
    // copy and use current driver's properties (returns information about read
    // driver, not saved data, i.e. tar driver says it is tar)
    auto p(detail().properties);
    p.driver = detail().driver->properties();
    return p;
}

Properties TileSet::setProperties(const SettableProperties &properties
                                  , SettableProperties::MaskType mask)
{
    detail().checkValidity();
    detail().driver->wannaWrite("set properties");

    // merge in new properties
    if (static_cast<SettableProperties&>
        (detail().properties).merge(properties, mask))
    {
        detail().propertiesChanged = true;
    }
    return detail().properties;
}

void TileSet::begin(utility::Runnable *runnable)
{
    detail().checkValidity();
    detail().begin(runnable);
}

void TileSet::commit()
{
    detail().checkValidity();
    detail().commit();
}

void TileSet::rollback()
{
    detail().checkValidity();
    detail().rollback();
}

bool TileSet::inTx() const
{
    return detail().tx;
}

void TileSet::Detail::clone(const Detail &src)
{
    // update properties
    properties.driver = driver->properties();
    properties.foat = src.properties.foat;
    properties.foatSize = src.properties.foatSize;
    propertiesChanged = true;

    const auto &sd(*src.driver);
    auto &dd(*driver);

    // copy single files
    for (auto type : { File::tileIndex  }) {
        copyFile(sd.input(type), dd.output(type));
    }

    // copy tiles
    traverseTiles(src.tileIndex, [&](const TileId &tileId)
    {
        for (auto type : { TileFile::mesh, TileFile::atlas }) {
            copyFile(sd.input(tileId, type), dd.output(tileId, type));
        }
    });

    // copy metatiles
    traverseTiles(src.metaIndex, [&](const TileId &metaId)
    {
        copyFile(sd.input(metaId, TileFile::meta)
                 , dd.output(metaId, TileFile::meta));
    });

    flush();

    // load index if we have anything
    if (savedProperties.foatSize) {
        // load tile index only if foat is valid
        loadTileIndex();
    }
}

void TileSet::Detail::clone(const Detail &src
                            , const boost::optional<Extents> &extents
                            , const boost::optional<LodRange> &lodRange)
{
    // update properties
    properties.driver = driver->properties();
    propertiesChanged = true;

    const auto &sd(*src.driver);
    auto &dd(*driver);

    // copy single files
    for (auto type : { File::config }) {
        copyFile(sd.input(type), dd.output(type));
    }

    // copy tiles
    const utility::Progress::ratio_t reportRatio(1, 100);
    utility::Progress progress(src.tileIndex.count());
    const auto name(str(boost::format("Cloning <%s> into <%s> ")
                        % src.properties.id % properties.id));
    traverseTiles(src.tileIndex, [&](const TileId &tileId)
    {
        if (lodRange && !in(*lodRange, tileId.lod)) {
            (++progress).report(reportRatio, name);
            return;
        }
        if (extents && !overlaps(*extents, tileExtents(properties, tileId))) {
            (++progress).report(reportRatio, name);
            return;
        }

        const auto *metanode(src.findMetaNode(tileId));
        if (!metanode) {
            LOG(warn2)
                << "Cannot find metanode for tile " << tileId << "; "
                << "skipping.";
            return;
        }

        // copy mesh and atlas
        for (auto type : { TileFile::mesh, TileFile::atlas }) {
            copyFile(sd.input(tileId, type), dd.output(tileId, type));
        }

        setMetaNode(tileId, *metanode);
        (++progress).report(reportRatio, name);
    });

    flush();

    // reload in new stuff
    loadConfig();
    if (savedProperties.foatSize) {
        // load tile index only if foat is valid
        loadTileIndex();
    }
}

void TileSet::Detail::dropRemovedMetatiles(const TileIndex &before
                                           , const TileIndex &after)
{
    // calculate difference between original and new state
    auto remove(difference(properties.alignment, before, after));

    {
        const auto *dumpRoot(getDumpDir());
        LOG(info1) << "dumpRoot: " << dumpRoot;
        dumpTileIndex(dumpRoot, "rm-before", before);
        dumpTileIndex(dumpRoot, "rm-after", after);
        dumpTileIndex(dumpRoot, "rm-remove", remove);
    }

    // copy metatiles
    traverseTiles(remove, [&](const TileId &metaId)
    {
        driver->remove(metaId, TileFile::meta);
    });
}

bool TileSet::empty() const
{
    // no foat -> no tile
    return !detail().properties.foatSize;
}

void TileSet::drop()
{
    detail().driver->drop();
    // make invalid!
    detail().driver.reset();
}

void TileSet::update()
{
    // tell driver to update its stuff
    detail().driver->wannaWrite("update tile set");
    detail().driver->update();
}

TileSet::AdvancedApi TileSet::advancedApi()
{
    detail().checkValidity();
    return TileSet::AdvancedApi(shared_from_this());
}

const TileIndex& TileSet::AdvancedApi::tileIndex() const
{
    const auto &detail(tileSet_->detail());
    detail.checkValidity();
    return detail.tileIndex ;
}

const TileIndex& TileSet::AdvancedApi::metaIndex() const
{
    const auto &detail(tileSet_->detail());
    detail.checkValidity();
    return detail.metaIndex ;
}

OStream::pointer TileSet::AdvancedApi::output(File type)
{
    auto &detail(tileSet_->detail());
    detail.checkValidity();
    return detail.driver->output(type);
}

IStream::pointer TileSet::AdvancedApi::input(File type) const
{
    const auto &detail(tileSet_->detail());
    detail.checkValidity();
    return detail.driver->input(type);
}

OStream::pointer TileSet::AdvancedApi::output(const TileId tileId
                                              , TileFile type)
{
    auto &detail(tileSet_->detail());
    detail.checkValidity();
    return detail.driver->output(tileId, type);
}

IStream::pointer TileSet::AdvancedApi::input(const TileId tileId
                                             , TileFile type) const
{
    const auto &detail(tileSet_->detail());
    detail.checkValidity();
    return detail.driver->input(tileId, type);
}

std::size_t TileSet::AdvancedApi::size(File type) const
{
    const auto &detail(tileSet_->detail());
    detail.checkValidity();
    return detail.driver->size(type);
}

std::size_t TileSet::AdvancedApi::size(const TileId tileId, TileFile type)
    const
{
    const auto &detail(tileSet_->detail());
    detail.checkValidity();
    return detail.driver->size(tileId, type);
}

void TileSet::AdvancedApi::regenerateTileIndex()
{
    // FIXME: not implemented so far

    // TODO: ensure that there are no pending changes
    auto &detail(tileSet_->detail());
    detail.checkValidity();

    if (!detail.savedProperties.foatSize) {
        // TODO: save empty index if nothing there
        return;
    }

    Metadata metadata;

    std::queue<TileId> subtrees({ detail.savedProperties.foat });

    while (!subtrees.empty()) {
        detail.loadMetatileFromFile
            (metadata, subtrees.front()
             , [&subtrees](const TileId &tileId) {subtrees.push(tileId); });
        subtrees.pop();
    }

    LOG(info3) << "Loaded " << metadata.size() << " nodes from metatiles.";
}

void TileSet::AdvancedApi::changeMetaLevels(const LodLevels &metaLevels)
{
    auto &detail(tileSet_->detail());

    // this action can be performed in transaction only
    detail.checkTx("change metalevels");

    LOG(info2)
        << "Trying to change metalevels from " << detail.properties.metaLevels
        << " to " << metaLevels << ".";

    if (detail.properties.metaLevels == metaLevels) {
        LOG(info2) << "Metalevels are same. No change.";
        return;
    }

    LOG(info2)
        << "Changing metalevels from " << detail.properties.metaLevels
        << " to " << metaLevels << ".";

    // set levels
    detail.properties.metaLevels = metaLevels;

    // properties has been changed
    detail.propertiesChanged = true;
    // force flush -> metatiles are about to be regenerated
    detail.metadataChanged = true;
}

void TileSet::AdvancedApi::changeSrs(const std::string& srs) 
{
    auto &detail(tileSet_->detail());

    // this action can be performed in transaction only
    detail.checkTx("change metalevels");

    LOG(info2)
        << "Trying to change SRS from " << detail.properties.srs
        << " to " << srs << ".";

    if (   detail.properties.srs != "" 
       && !geo::areSame(detail.properties.srs, srs)) 
    {
        LOGTHROW(err3, std::logic_error) << "Unable to change SRS.";
    }

    LOG(info2)
        << "Changing SRS from " << detail.properties.srs
        << " to " << srs << ".";

    // set levels
    detail.properties.srs = srs;

    // properties has been changed
    detail.propertiesChanged = true;
    // force flush -> mapConfig is about to be regenerated
    detail.metadataChanged = true;
}

void TileSet::AdvancedApi::rename(const std::string &newId)
{
    auto &detail(tileSet_->detail());
    detail.checkValidity();

    if (detail.properties.id == newId) {
        return;
    }

    detail.properties.id = newId;

    // propetries has been changed
    detail.propertiesChanged = true;
}


void TileSet::AdvancedApi::removeOutOfLodRange( const TileId &tileId
                                             , const LodRange & lodRange )
{
    auto &detail(tileSet_->detail());

    if(auto *node = detail.findMetaNode(tileId)){
        (void ) node;
        for (const auto &childId : children(detail.properties.baseTileSize, tileId)) {
            removeOutOfLodRange(childId, lodRange);
        }
        //if leaf node and outside of the extents, remove tile
        if(tileId.lod < lodRange.min || tileId.lod > lodRange.max){
            LOG(info2)<< "Removing tile "<<tileId;
            tileSet_->removeTile(tileId);
        }
    }
}

void TileSet::AdvancedApi::removeOutOfLodRange( const LodRange & lodRange )
{
    auto &detail(tileSet_->detail());

    removeOutOfLodRange( detail.properties.foat, lodRange );

    detail.metadataChanged = true;
}

void TileSet::AdvancedApi::removeOutOfExtents( const TileId &tileId
                                             , const math::Extents2 & extents )
{
    auto &detail(tileSet_->detail());

    if(auto *node = detail.findMetaNode(tileId)){
        (void ) node;
        for (const auto &childId : children(detail.properties.baseTileSize, tileId)) {
            removeOutOfExtents(childId, extents);
        }
        //if leaf node and outside of the extents, remove tile
        if(!math::overlaps(tileExtents(detail.properties.baseTileSize, tileId), extents )){
            LOG(info2)<< "Removing tile "<<tileId;
            tileSet_->removeTile(tileId);
        }
    }
}

void TileSet::AdvancedApi::removeOutOfExtents( const math::Extents2 & extents )
{
    auto &detail(tileSet_->detail());

    removeOutOfExtents( detail.properties.foat, extents );

    detail.metadataChanged = true;
}


void TileSet::AdvancedApi::forceMetadata( const TileId tileId
                                        , const TileMetadata &metadata
                                        , const TileMetadata::MaskType mask)
{
    auto &detail(tileSet_->detail());


    if(auto *node = detail.findMetaNode(tileId)){
        if(mask & TileMetadata::Mask::gsd){
            node->gsd = metadata.gsd;
        }

        if(mask & TileMetadata::Mask::coarseness){
            node->coarseness = metadata.coarseness;
        }

        for (const auto &childId : children(detail.properties.baseTileSize, tileId)) {
            forceMetadata(childId, metadata, mask);
        }
    };
}

void TileSet::AdvancedApi::forceMetadata( const TileMetadata &metadata
                                        , const TileMetadata::MaskType mask)
{
    (void) mask;
    auto &detail(tileSet_->detail());

    forceMetadata( detail.properties.foat, metadata, mask );

    detail.metadataChanged = true;
}

bool TileSet::compatible(const TileSet &other)
{
    const auto props(detail().properties);

    const auto oDetail(other.detail());
    const auto oProps(oDetail.properties);

    if ( !geo::areSame(oProps.srs, props.srs) ) {
         LOG(warn2)
            << "Tile set <" << props.id
            << ">: set <" << oProps.id
            << "> has incompatible SRS.";
         return false;
    }

    if (oProps.baseTileSize != props.baseTileSize) {
        LOG(warn2)
            << "Tile set <" << props.id
            << ">: set <" << oProps.id
            << "> has incompatible base tile size.";
        return false;
    }

    if (oProps.alignment == props.alignment) {
        // same alignment -> fine
        return true;
    }

    // different alignment: we have to check min-lod alignment compatibility
    auto ts(tileSize(oProps, oDetail.lodRange.min));

    auto diff(props.alignment - oProps.alignment);
    if ((diff(0) % ts) || (diff(1) % ts)) {
        LOG(warn2)
            << "Tile set <" << props.id
            << ">: set <" << oProps.id
            << "> has incompatible alignment.";
        return false;
    }
    return true;
}

Extents TileSet::extents() const {return detail().extents; }

LodRange TileSet::lodRange() const {return detail().lodRange; }

void pasteTileSets(const TileSet::pointer &dst
                   , const TileSet::list &src)
{
    // paste tiles
    dst->paste(src);

    // creates and immediately commits a transaction -> generates metadata in tx
    // that is flushed and commited
    if (!dst->inTx()) {
        // no pending transaction -> create one :)
        dst->begin();
        dst->commit();
    } else {
        // just flush
        dst->flush();
    }
}

} } // namespace vadstena::tilestorage
