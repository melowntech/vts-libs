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
    return detail().getMesh(tileId);
}

void TileSet::getAtlas(const TileId &tileId, Atlas &atlas) const
{
    detail().getAtlas(tileId, atlas);
}

void TileSet::getNavTile(const TileId &tileId, NavTile &navtile) const
{
    detail().getNavTile(tileId, navtile);
}

void TileSet::setTile(const TileId &tileId, const Tile &tile)
{
    detail().setTile(tileId, tile.mesh.get(), tile.atlas.get()
                     , tile.navtile.get());
}

MetaNode TileSet::getMetaNode(const TileId &tileId) const
{
    auto *node(detail().findNode(tileId));
    if (!node) {
        LOGTHROW(err2, storage::NoSuchTile)
            << "There is no tile at " << tileId << ".";
    }
    return *node->metanode;
}

bool TileSet::exists(const TileId &tileId) const
{
    return detail().exists(tileId);
}

void TileSet::flush()
{
    detail().flush();
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
    : readOnly(true), driver(driver), changed(false)
{
    loadConfig();
    referenceFrame = registry::Registry::referenceFrame
        (properties.referenceFrame);

    loadTileIndex();
}

TileSet::Detail::Detail(const Driver::pointer &driver
                        , const StaticProperties &properties)
    : readOnly(false), driver(driver), changed(false)
    , referenceFrame(registry::Registry::referenceFrame
                     (properties.referenceFrame))
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

    // save config and (empty) tile indices
    saveConfig();
}

TileSet::Detail::~Detail()
{
    // no exception thrown and not flushed? warn user!
    if (!std::uncaught_exception() && changed) {
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
        properties = p;
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
}

void TileSet::Detail::loadTileIndex()
{
    try {
        tileIndex = {};
        auto f(driver->input(File::tileIndex));
        tileIndex.load(*f);
        f->close();
    } catch (const std::exception &e) {
        LOGTHROW(err2, storage::Error)
            << "Unable to read tile index: " << e.what() << ".";
    }

    // new extents
    lodRange = tileIndex.lodRange();
    LOG(debug) << "Loaded tile index: " << tileIndex;
}

void TileSet::Detail::saveTileIndex()
{
    try {
        auto f(driver->output(File::tileIndex));
        tileIndex.save(*f);
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
        const MetaNode *node(nullptr);
        if (!addNew) {
            node = meta->get(tileId, std::nothrow);
            if (!node) { return nullptr; }
        } else {
            // add new node
            node = meta->set(tileId, {});
        }

        // add node to the tree
        return &tileNodes.insert
            (TileNode::map::value_type
             (tileId, TileNode(meta, node)))
            .first->second;
    }

    return &ftileNodes->second;
}

MetaTile* TileSet::Detail::addNewMetaTile(const TileId &tileId) const
{
    auto mid(metaId(tileId));
    LOG(info1) << "Creating metatile " << mid << ".";
    return &metaTiles.insert
        (MetaTiles::value_type
         (mid, MetaTile(mid,  metaOrder()))).first->second;
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
        if (!tileIndex.checkMask(mid, TileFlag::meta)) {
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

registry::ReferenceFrame TileSet::referenceFrame() const
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

void TileSet::Detail::load(const IStream::pointer &os, Mesh &mesh) const
{
    mesh = loadMesh(*os, os->name());
}

void TileSet::Detail::load(const IStream::pointer &os, Atlas &atlas) const
{
    atlas.deserialize(os->get(), os->name());
}

void TileSet::Detail::load(const NavTile::HeightRange &heightRange
                           , const IStream::pointer &os
                           , NavTile &navtile)
    const
{
    navtile.deserialize(heightRange, os->get(), os->name());
}

TileNode* TileSet::Detail::updateNode(TileId tileId
                                      , const MetaNode &metanode
                                      , bool watertight)
{
    // get node (create if necessary)
    auto *node(findNode(tileId, true));

    // update node value
    node->update(tileId, metanode);

    tileIndex.setMask(tileId, TileFlag::watertight, watertight);

    // go up the tree
    while (tileId.lod) {
        auto parentId(parent(tileId));
        auto *parentNode = findNode(parentId, true);

        auto mn(*parentNode->metanode);
        parentNode->set(parentId, mn.setChildFromId(tileId)
                        .mergeExtents(*node->metanode));

        // next round
        tileId = parentId;
        node = parentNode;
    }

    return node;
}

void accumulateBoundLayers(registry::IdSet &ids, const Mesh &mesh)
{
    for (const auto &sm : mesh) {
        if (sm.textureLayer) { ids.insert(*sm.textureLayer); }
    }
}

void TileSet::Detail::setTile(const TileId &tileId
                              , const Mesh *mesh
                              , const Atlas *atlas
                              , const NavTile *navtile)
{
    driver->wannaWrite("set tile");

    LOG(info1) << "Setting content of tile " << tileId << ".";

    MetaNode metanode;

    // set various flags and metadata
    if (mesh) {
        // geometry
        metanode.geometry(true);
        metanode.extents = normalizedExtents(referenceFrame, extents(*mesh));

        // get external textures info configuration
        accumulateBoundLayers(properties.boundLayers, *mesh);

        // mesh and texture area stuff
        auto ma(area(*mesh));
        metanode.cc(MetaNode::CoarsenessControl::texelSize);
        metanode.meshArea = std::sqrt(ma.mesh);

        // internal texture
        metanode.internalTexture(true);

        // calculate texture area
        double textureArea(0.0);
        {
            std::size_t index(0);
            auto isubmeshes(mesh->submeshes.begin());
            for (double ta : ma.texture) {
                if (atlas) {
                    textureArea += ta * atlas->area(index++);
                } else if (isubmeshes->textureLayer) {
                    // TODO: use value of to get size of texture textureLayer
                    auto ts(registry::Registry::boundLayer
                            (*isubmeshes->textureLayer).tileSize);
                    textureArea += ta * area(ts);
                }
            }
            ++isubmeshes;
        }
        metanode.textureArea = std::sqrt(textureArea);
    }

    // navtile
    if (navtile) {
        metanode.navtile(true);
        metanode.heightRange = navtile->heightRange();
    }

    // store node
    updateNode(tileId, metanode, watertight(mesh));

    // save data
    if (mesh) {
        save(driver->output(tileId, TileFile::mesh), *mesh);
    }
    if (atlas) {
        save(driver->output(tileId, TileFile::atlas), *atlas);
    }

    if (navtile) {
        save(driver->output(tileId, TileFile::navtile), *navtile);
    }
}

Mesh TileSet::Detail::getMesh(const TileId &tileId) const
{
    auto *node(findNode(tileId));
    if (!node) {
        LOGTHROW(err2, storage::NoSuchTile)
            << "There is no tile at " << tileId << ".";
    }

    Mesh mesh;
    load(driver->input(tileId, TileFile::mesh), mesh);
    return mesh;
}

void TileSet::Detail::getAtlas(const TileId &tileId, Atlas &atlas) const
{
    auto *node(findNode(tileId));
    if (!node) {
        LOGTHROW(err2, storage::NoSuchTile)
            << "There is no tile at " << tileId << ".";
    }

    if (!node->metanode->internalTexture()) {
        LOGTHROW(err2, storage::NoSuchTile)
            << "Tile " << tileId << " has no atlas.";
    }

    load(driver->input(tileId, TileFile::atlas), atlas);
}

void TileSet::Detail::getNavTile(const TileId &tileId, NavTile &navtile) const
{
    auto *node(findNode(tileId));
    if (!node) {
        LOGTHROW(err2, storage::NoSuchTile)
            << "There is no tile at " << tileId << ".";
    }

    if (!node->metanode->navtile()) {
        LOGTHROW(err2, storage::NoSuchTile)
            << "Tile " << tileId << " has no navtile.";
    }

    load(node->metanode->heightRange
         , driver->input(tileId, TileFile::navtile)
         , navtile);
}

bool TileSet::Detail::exists(const TileId &tileId) const
{
    // first try index
    if (tileIndex.exists(tileId)) { return true; }

    // then check for in-memory data
    return (tileNodes.find(tileId) != tileNodes.end());
}

void TileSet::Detail::saveMetadata()
{
    driver->wannaWrite("save metadata");

    for (const auto &item : metaTiles) {
        const auto &meta(item.second);
        auto tileId(meta.origin());

        {
            // save metatile to file
            auto f(driver->output(tileId, TileFile::meta));
            meta.save(*f);
            f->close();
        }

        meta.for_each([&](const TileId &tileId, const MetaNode &node)
        {
            // mark node only if real
            if (node.real()) {
                // TODO: convert node info into mask
                tileIndex.setMask(tileId, 1);
            }
        });

        // mark metatile
        tileIndex.setMask(tileId, TileFlag::meta);
    }

    saveTileIndex();
}

void update(TileSet::Properties &properties, const TileIndex &tileIndex)
{
    auto stat(tileIndex.statMask(TileSet::Detail::TileFlag::mesh
                                 | TileSet::Detail::TileFlag::atlas));
    properties.lodRange = stat.lodRange;
    if (properties.lodRange.empty()) {
        properties.tileRange = TileRange(math::InvalidExtents{});
    } else {
        properties.tileRange = stat.tileRanges.front();
    }
}

void TileSet::Detail::flush()
{
    driver->wannaWrite("flush");

    saveMetadata();

    // update and save config
    update(properties, tileIndex);
    saveConfig();

    // flush driver
    driver->flush();
}

const Driver& TileSet::driver() const
{
    return *detail().driver;
}

MapConfig TileSet::mapConfig() const
{
    return detail().mapConfig();
}

MapConfig TileSet::Detail::mapConfig() const
{
    MapConfig mapConfig;

    // TODO: load extra config
    // TODO: fill-in data from both configs and registry

    mapConfig.referenceFrame = referenceFrame;
    mapConfig.srs = registry::listSrs(referenceFrame);
    mapConfig.credits = registry::creditsAsDict(properties.credits);
    mapConfig.boundLayers
        = registry::boundLayersAsDict(properties.boundLayers);

    mapConfig.surfaces.emplace_back();
    auto &surface(mapConfig.surfaces.back());
    surface.id = properties.id;

    // local path
    surface.root = fs::path();

    if (!properties.lodRange.empty()) {
        surface.lodRange = properties.lodRange;
    }

    if (valid(properties.tileRange)) {
        surface.tileRange = properties.tileRange;
    }

    return mapConfig;
}

} } // namespace vadstena::vts
