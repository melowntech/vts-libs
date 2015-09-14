#include "../../vts.hpp"
#include "../tileset.hpp"
#include "../tileindex-io.hpp"
#include "../io.hpp"
#include "./detail.hpp"
#include "./driver.hpp"
#include "./config.hpp"

namespace fs = boost::filesystem;

namespace vadstena { namespace vts {

StaticProperties TileSet::getProperties() const
{
    return {};
}

TileSet::TileSet(const std::shared_ptr<Driver> &driver)
    : detail_(new Detail(driver))
{
}

TileSet::TileSet(const std::shared_ptr<Driver> &driver
                 , const StaticProperties &properties)
    : detail_(new Detail(driver, properties))
{
}

Mesh TileSet::getMesh(const TileId &tileId) const
{
    (void) tileId;
    return {};
}

void TileSet::setMesh(const TileId &tileId, const Mesh &mesh
                      , bool watertight)
{
    (void) tileId;
    (void) mesh;
    (void) watertight;

    auto *node(detail().findNode(tileId));
    LOG(info4) << "Found meta node: " << node;
    (void) node;
}

void TileSet::getAtlas(const TileId &tileId, Atlas &atlas) const
{
    (void) tileId; (void) atlas;
}

void TileSet::setAtlas(const TileId &tileId, const Atlas &atlas)
{
    (void) tileId; (void) atlas;
}

void TileSet::setTile(const TileId &tileId, const Tile &tile)
{
    detail().setTile(tileId, tile);
}

MetaNode TileSet::getMetaNode(const TileId &tileId) const
{
    (void) tileId;
    return {};
}

void TileSet::setMetaNode(const TileId &tileId, const MetaNode node)
{
    (void) tileId;
    (void) node;
}

bool TileSet::exists(const TileId &tileId) const
{
    (void) tileId;
    return false;
}

void TileSet::flush()
{
    // TODO: write metadata to storage

    detail().driver->flush();
}

void TileSet::watch(utility::Runnable *runnable)
{
    detail().checkValidity();
    detail().watch(runnable);
}

bool TileSet::empty() const
{
    return true;
}

void TileSet::drop()
{
    detail().driver->drop();
    // make invalid!
    detail().driver.reset();
}

LodRange TileSet::lodRange() const
{
    return {};
}

struct TileSet::Factory
{
    static TileSet create(const fs::path &path
                          , const StaticProperties &properties
                          , CreateMode mode)
    {
        // we are using binaryOrder = 5 :)
        auto driver(std::make_shared<Driver>(path, mode, 5));
        return TileSet(driver, properties);
    }

    static TileSet open(const fs::path &path)
    {
        auto driver(std::make_shared<Driver>(path));
        return TileSet(driver);
    }
};

TileSet createTileSet(const boost::filesystem::path &path
                      , const StaticProperties &properties
                      , CreateMode mode)
{
    return TileSet::Factory::create(path, properties, mode);
}

TileSet openTileSet(const boost::filesystem::path &path)
{
    return TileSet::Factory::open(path);
}

TileSet::Detail::Detail(const Driver::pointer &driver)
    : readOnly(true), driver(driver), propertiesChanged(false)
    , metadataChanged(false)
{
    loadConfig();
    referenceFrame = storage::Registry::referenceFrame
        (properties.referenceFrame);

    loadTileIndex();
}

TileSet::Detail::Detail(const Driver::pointer &driver
                        , const StaticProperties &properties)
    : readOnly(false), driver(driver), propertiesChanged(false)
    , referenceFrame(storage::Registry::referenceFrame
                     (properties.referenceFrame))
    , metadataChanged(false)
    , lodRange(LodRange::emptyRange())
{
    if (properties.id.empty()) {
        LOGTHROW(err2, storage::FormatError)
            << "Cannot create tile set without valid id.";
    }

    // build initial properties
    static_cast<StaticProperties&>(this->properties) = properties;
    this->properties.driverOptions = driver->options();

    if (auto oldConfig = driver->oldConfig()) {
        try {
            // try to old config and grab old revision
            std::istringstream is(*oldConfig);
            const auto p(vts::loadConfig(is));
            this->properties.revision = p.revision + 1;
        } catch (...) {}
    }
    savedProperties = this->properties;

    // save config and (empty) tile indices
    saveConfig();
    saveTileIndex();
}

TileSet::Detail::~Detail()
{
    // no exception thrown and not flushed? warn user!
    if (!std::uncaught_exception()
        && (metadataChanged || propertiesChanged))
    {
        LOG(warn3)
            << "Tile set <" << properties.id
            << "> is not flushed on destruction: data could be unusable.";
    }
}

TileSet::~TileSet() = default;

void TileSet::Detail::loadConfig()
{
    try {
        // load config
        auto f(driver->input(File::config));
        const auto p(vts::loadConfig(*f));
        f->close();

        // set
        savedProperties = properties = p;
    } catch (const std::exception &e) {
        LOGTHROW(err2, storage::Error)
            << "Unable to read config: <" << e.what() << ">.";
    }
}

void TileSet::Detail::saveConfig()
{
    // save json
    try {
        driver->wannaWrite("save config");
        auto f(driver->output(File::config));
        vts::saveConfig(*f, properties);
        f->close();
    } catch (const std::exception &e) {
        LOGTHROW(err2, storage::Error)
            << "Unable to write config: <" << e.what() << ">.";
    }

    // done; remember saved properties and go on
    savedProperties = properties;
    propertiesChanged = false;
}

void TileSet::Detail::loadTileIndex()
{
    try {
        tileIndex = {};
        metaIndex = {};
        auto f(driver->input(File::tileIndex));
        tileIndex.load(*f);
        watertightIndex.load(*f);
        metaIndex.load(*f);
        f->close();
    } catch (const std::exception &e) {
        LOGTHROW(err2, storage::Error)
            << "Unable to read tile index: " << e.what() << ".";
    }

    // new extents
    lodRange = tileIndex.lodRange();
    LOG(info2) << "Loaded tile index: " << tileIndex;
}

void TileSet::Detail::saveTileIndex()
{
    try {
        auto f(driver->output(File::tileIndex));
        tileIndex.save(*f);
        watertightIndex.save(*f);
        metaIndex.save(*f);
        f->close();
    } catch (const std::exception &e) {
        LOGTHROW(err2, storage::Error)
            << "Unable to read tile index: " << e.what() << ".";
    }
}

void TileSet::Detail::watch(utility::Runnable *runnable)
{
    driver->watch(runnable);
}

TileNode* TileSet::Detail::findNode(const TileId &tileId, bool addNew)
    const
{
    auto ftileNodes(tileNodes.find(tileId));
    if (ftileNodes == tileNodes.end()) {
        // not found in memory -> load from disk
        auto *meta(findMetaTile(tileId, addNew));
        if (!meta) { return nullptr; }

        // now, find node in the metatile
        auto *node(meta->get(tileId, std::nothrow));
        if (!node) { return nullptr; }

        LOG(info1) << "Creating metanode " << tileId << ".";
        // add node to the tree
        return &tileNodes.insert
            (TileNode::map::value_type
             (tileId, TileNode(node, watertightIndex.exists(tileId))))
            .first->second;
    }

    return &ftileNodes->second;
}

MetaTile* TileSet::Detail::addNewMetaTile(const TileId &tileId) const
{
    auto mid(metaId(tileId));
    auto tid(originFromMetaId(mid));
    LOG(info1) << "Creating metatile " << mid << " with origin " << tid << ".";
    return &metaTiles.insert
        (MetaTiles::value_type
         (mid, MetaTile(tid,  metaOrder()))).first->second;
}

MetaTile* TileSet::Detail::findMetaTile(const TileId &tileId, bool addNew)
    const
{
    TileId mid(metaId(tileId));

    // try to find metatile
    auto fmetaTiles(metaTiles.find(mid));

    // load metatile if not found
    if (fmetaTiles == metaTiles.end()) {
        // does this metatile exist in the index?
        if (!metaIndex.exists(mid)) {
            if (addNew) { return addNewMetaTile(tileId); }
            return nullptr;
        }

        // it should be on the disk!
        auto f(driver->input(mid, TileFile::meta));
        fmetaTiles = metaTiles.insert
            (MetaTiles::value_type
             (mid, loadMetaTile(*f, metaOrder()))).first;
    }

    return &fmetaTiles->second;
}

storage::ReferenceFrame TileSet::referenceFrame() const
{
    return detail().referenceFrame;
}


void TileSet::Detail::save(const OStream::pointer &os, const Mesh &mesh) const
{
    saveMesh(*os, mesh);
    os->close();
}

void TileSet::Detail::save(const OStream::pointer &os, const Atlas &atlas)
    const
{
    atlas.serialize(os->get());
    os->close();
}

void TileSet::Detail::save(const OStream::pointer &os, const NavTile &navtile)
    const
{
    navtile.serialize(os->get());
    os->close();
}

TileNode* TileSet::Detail::updateNode(TileId tileId
                                      , const MetaNode &metanode)
{
    // get node (create if necessary)
    auto *node(findNode(tileId, true));

    // update node value
    node->metanode->update(metanode);

    // go up the tree
    while (tileId.lod) {
        tileId = parent(tileId);
        node = findNode(tileId, true);

        // TODO: update child flags
    }

    return node;
}

void TileSet::Detail::setTile(const TileId &tileId, const Tile &tile)
{
    driver->wannaWrite("set tile");

    LOG(info1) << "Setting content of tile " << tileId << ".";

    MetaNode metanode;

    // set various flags and metadata
    // geometry
    metanode.geometry(true);
    metanode.extents = normalizedExtents(referenceFrame, extents(tile.mesh));

    auto ma(area(tile.mesh));
    metanode.cc(MetaNode::CoarsenessControl::texelSize);
    metanode.meshArea = std::sqrt(ma.mesh);

    // internal texture
    if (tile.atlas) {
        metanode.internalTexture(true);
        // calculate texture area

        double total(0.0);
        std::size_t index(0);
        for (double ta : ma.texture) {
            total += ta * tile.atlas->area(index++);
        }

        metanode.textureArea = std::sqrt(total);
    }

    // navtile
    if (tile.navtile) {
        metanode.navtile(true);
        metanode.heightRange = tile.navtile->heightRange();
    }

    // store node
    updateNode(tileId, metanode);

    // save data
    save(driver->output(tileId, TileFile::mesh), tile.mesh);
    if (tile.atlas) {
        save(driver->output(tileId, TileFile::atlas), *tile.atlas);
    }
    if (tile.navtile) {
        save(driver->output(tileId, TileFile::navtile), *tile.navtile);
    }
}

} } // namespace vadstena::vts
