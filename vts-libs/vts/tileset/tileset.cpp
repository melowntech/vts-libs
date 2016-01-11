#include <boost/format.hpp>
#include <boost/range/adaptor/reversed.hpp>

#include "utility/progress.hpp"
#include "geo/csconvertor.hpp"

#include "../../vts.hpp"
#include "../tileset.hpp"
#include "../tileindex-io.hpp"
#include "../io.hpp"
#include "./detail.hpp"
#include "./driver.hpp"
#include "./config.hpp"

namespace fs = boost::filesystem;

namespace vadstena { namespace vts {

TileSetProperties TileSet::getProperties() const
{
    return detail().properties;
}

void TileSet::setPosition(const registry::Position &position)
{
    detail().setPosition(position);
}

void TileSet::addCredits(const registry::IdSet &credits)
{
    detail().addCredits(credits);
}

void TileSet::addBoundLayers(const registry::IdSet &boundLayers)
{
    detail().addBoundLayers(boundLayers);
}

TileSet::TileSet(const std::shared_ptr<Driver> &driver)
    : detail_(std::make_shared<Detail>(driver))
{
}

TileSet::TileSet(const std::shared_ptr<Driver> &driver
                 , const TileSet::Properties &properties)
    : detail_(std::make_shared<Detail>(driver, properties))
{}

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

TileSource TileSet::getTileSource(const TileId &tileId) const
{
    return detail().getTileSource(tileId);
}

void TileSet::setTile(const TileId &tileId, const Tile &tile)
{
    detail().setTile(tileId, tile);
}

void TileSet::setTile(const TileId &tileId, const Tile &tile
                      , const NodeInfo &nodeInfo)
{
    detail().setTile(tileId, tile, &nodeInfo);
}

void TileSet::setTile(const TileId &tileId, const TileSource &tile)
{
    detail().setTile(tileId, tile);
}

void TileSet::setTile(const TileId &tileId, const TileSource &tile
                      , const NodeInfo &nodeInfo)
{
    detail().setTile(tileId, tile, &nodeInfo);
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

MetaTile TileSet::getMetaTile(const TileId &metaId) const
{
    auto *mt(detail().findMetaTile(metaId));
    if (!mt) {
        LOGTHROW(err2, storage::NoSuchTile)
            << "There is no metatile at " << metaId << ".";
    }
    return *mt;
}

bool TileSet::exists(const TileId &tileId) const
{
    return detail().exists(tileId);
}

bool TileSet::fullyCovered(const TileId &tileId) const
{
    return detail().fullyCovered(tileId);
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
    return detail().tileIndex.empty();
}

void TileSet::drop()
{
    detail().driver->drop();
    // make invalid!
    detail().driver.reset();
}

LodRange TileSet::lodRange() const
{
    return detail().properties.lodRange;
}

struct TileSet::Factory
{
    static TileSet create(const fs::path &path
                          , const TileSet::Properties &properties
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

    static void clone(const std::string &reportName
                      , const Detail &src, Detail &dst)
    {
        const auto &sd(*src.driver);
        auto &dd(*dst.driver);

        copyFile(sd.input(storage::File::tileIndex)
                 , dd.output(storage::File::tileIndex));

        const utility::Progress::ratio_t reportRatio(1, 100);
        utility::Progress progress(src.tileIndex.count());
        auto report([&]() { (++progress).report(reportRatio, reportName); });

        traverse(src.tileIndex, [&](const TileId &tid, QTree::value_type mask)
        {
            if (mask & TileIndex::Flag::mesh) {
                // copy mesh
                copyFile(sd.input(tid, storage::TileFile::mesh)
                         , dd.output(tid, storage::TileFile::mesh));
            }

            if (mask & TileIndex::Flag::atlas) {
                // copy atlas
                copyFile(sd.input(tid, storage::TileFile::atlas)
                         , dd.output(tid, storage::TileFile::atlas));
            }

            if (mask & TileIndex::Flag::meta) {
                // copy meta
                copyFile(sd.input(tid, storage::TileFile::meta)
                         , dd.output(tid, storage::TileFile::meta));
            }

            if (mask & TileIndex::Flag::navtile)
            {
                // copy navtile if allowed
                copyFile(sd.input(tid, storage::TileFile::navtile)
                         , dd.output(tid, storage::TileFile::navtile));
            }

            LOG(info1) << "Stored tile " << tid << ".";
            report();
        });

        // load copied tile index
        dst.loadTileIndex();

        // properties have been changed
        dst.propertiesChanged = true;
    }

    static void clone(const std::string &reportName
                      , const Detail &src, Detail &dst
                      , const LodRange &lodRange)
    {
        const auto &sd(*src.driver);
        auto &dd(*dst.driver);

        const utility::Progress::ratio_t reportRatio(1, 100);
        utility::Progress progress(src.tileIndex.count());
        auto report([&]() { (++progress).report(reportRatio, reportName); });

        traverse(src.tileIndex, [&](const TileId &tid, QTree::value_type mask)
        {
            // skip
            if (!in(lodRange, tid.lod)) {
                report();
                return;
            }

            const auto *metanode(src.findMetaNode(tid));
            if (!metanode) {
                if (mask & TileIndex::Flag::content) {
                    LOG(warn2)
                        << "Cannot find metanode for tile " << tid << "; "
                        << "skipping (flags: " << std::bitset<8>(mask)
                        << ").";
                }
                report();
                return;
            }

            if (mask & TileIndex::Flag::mesh) {
                // copy mesh
                copyFile(sd.input(tid, storage::TileFile::mesh)
                         , dd.output(tid, storage::TileFile::mesh));
            }

            if (mask & TileIndex::Flag::atlas) {
                // copy atlas
                copyFile(sd.input(tid, storage::TileFile::atlas)
                         , dd.output(tid, storage::TileFile::atlas));
            }

            if (mask & TileIndex::Flag::navtile)
            {
                // copy navtile if allowed
                copyFile(sd.input(tid, storage::TileFile::navtile)
                         , dd.output(tid, storage::TileFile::navtile));
            }

            dst.updateNode(tid, *metanode
                           , mask & TileIndex::Flag::watertight);
            LOG(info1) << "Stored tile " << tid << ".";
            report();
        });

        // properties have been changed
        dst.propertiesChanged = true;
    }

    static TileSet clone(const boost::filesystem::path &path
                         , const TileSet &src
                         , const CloneOptions &cloneOptions)
    {
        auto properties(src.detail().properties);
        if (cloneOptions.tilesetId()) {
            properties.id = *cloneOptions.tilesetId();
        }
        auto dst(TileSet::Factory::create(path, properties
                                          , cloneOptions.mode()));

        const auto reportName(str(boost::format("Cloning <%s> ")
                                  % src.detail().properties.id));
        if (cloneOptions.lodRange()) {
            clone(reportName, src.detail(), dst.detail()
                  , *cloneOptions.lodRange());
        } else {
            clone(reportName, src.detail(), dst.detail());
        }

        // and flush
        dst.flush();

        return dst;
    }
};

TileSet createTileSet(const boost::filesystem::path &path
                      , const TileSetProperties &properties
                      , CreateMode mode)
{
    return TileSet::Factory::create(path, properties, mode);
}

TileSet openTileSet(const boost::filesystem::path &path)
{
    return TileSet::Factory::open(path);
}

TileSet cloneTileSet(const boost::filesystem::path &path, const TileSet &src
                     , const CloneOptions &cloneOptions)
{
    return TileSet::Factory::clone(path, src, cloneOptions);
}

TileSet::Detail::Detail(const Driver::pointer &driver)
    : readOnly(true), driver(driver)
    , propertiesChanged(false), metadataChanged(false)
{
    loadConfig();
    referenceFrame = registry::Registry::referenceFrame
        (properties.referenceFrame);

    loadTileIndex();
}

TileSet::Detail::Detail(const Driver::pointer &driver
                        , const TileSet::Properties &properties)
    : readOnly(false), driver(driver)
    , propertiesChanged(false), metadataChanged(false)
    , referenceFrame(registry::Registry::referenceFrame
                     (properties.referenceFrame))
    , lodRange(LodRange::emptyRange())
{
    if (properties.id.empty()) {
        LOGTHROW(err2, storage::FormatError)
            << "Cannot create tile set without valid id.";
    }

    // build initial properties
    this->properties = properties;
    this->properties.driverOptions = driver->options();

    if (auto oldConfig = driver->oldConfig()) {
        try {
            // try to old config and grab old revision
            std::istringstream is(*oldConfig);
            const auto p(tileset::loadConfig(is));
            this->properties.revision = p.revision + 1;
        } catch (...) {}
    }

    // save config and (empty) tile index and reference
    saveConfig();
    saveTileIndex();
}

TileSet::Detail::~Detail()
{
    // no exception thrown and not flushed? warn user!
    if (!std::uncaught_exception() && changed()) {
        LOG(warn3)
            << "Tile set <" << properties.id
            << "> is not flushed on destruction: data could be unusable.";
    }
}

TileSet::~TileSet() = default;

void TileSet::Detail::loadConfig()
{
    properties = loadConfig(*driver);
}

TileSet::Properties TileSet::Detail::loadConfig(const Driver &driver)
{
    try {
        // load config
        auto f(driver.input(File::config));
        const auto p(tileset::loadConfig(*f));
        f->close();

        // set
        return p;
    } catch (const std::exception &e) {
        LOGTHROW(err2, storage::Error)
            << "Unable to read config: <" << e.what() << ">.";
    }
    throw;
}

void TileSet::Detail::saveConfig()
{
    // save json
    try {
        driver->wannaWrite("save config");
        auto f(driver->output(File::config));
        tileset::saveConfig(*f, properties);
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

        if (f->get().peek() != std::istream::traits_type::eof()) {
            references.load(*f);
        }
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
        references.save(*f);
        f->close();
    } catch (const std::exception &e) {
        LOGTHROW(err2, storage::Error)
            << "Unable to save tile index: " << e.what() << ".";
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
    metadataChanged = true;
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
        if (!tileIndex.checkMask(mid, TileIndex::Flag::meta)) {
            if (addNew) { return addNewMetaTile(tileId); }
            return nullptr;
        }

        // it should be on the disk!
        auto f(driver->input(mid, TileFile::meta));
        fmetaTiles = metaTiles.insert
            (MetaTiles::value_type
             (mid, loadMetaTile(*f, metaOrder(), f->name()))).first;
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

void TileSet::Detail::updateProperties(const MetaNode &metanode)
{
    properties.credits.insert(metanode.credits().begin()
                              , metanode.credits().end());
    propertiesChanged = true;
}

void TileSet::Detail::updateProperties(const NodeInfo &nodeInfo)
{
    auto res(properties.spatialDivisionExtents.insert
             (SpatialDivisionExtents::value_type
              (nodeInfo.subtreeRoot->srs, nodeInfo.node.extents)));
    if (!res.second) {
        res.first->second
            = math::unite(res.first->second, nodeInfo.node.extents);
        propertiesChanged = true;
    }
}

void TileSet::Detail::updateProperties(const Mesh &mesh)
{
    for (const auto &sm : mesh) {
        if (sm.textureLayer) {
            properties.boundLayers.insert(*sm.textureLayer);
        }
    }
}

TileNode* TileSet::Detail::updateNode(TileId tileId
                                      , const MetaNode &metanode
                                      , bool watertight)
{
    // get node (create if necessary)
    auto *node(findNode(tileId, true));

    // update node value
    node->update(tileId, metanode);

    tileIndex.setMask(tileId, TileIndex::Flag::watertight, watertight);
    if (auto r = node->metanode->reference()) {
        tileIndex.setMask(tileId, TileIndex::Flag::reference, true);
        references.set(tileId, r);
    }

    // collect global information (i.e. credits)
    updateProperties(metanode);

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

bool check(const SpatialDivisionExtents &l, const SpatialDivisionExtents &r)
{
    auto il(l.begin()), el(l.end());
    auto ir(r.begin()), er(r.end());

    // process in parallel
    while ((il != el) && (ir != er)) {
        if (il->first < ir->first) {
            // left behind right
            ++il;
        } else if (ir->first < il->first) {
            // right behind left
            ++ir;
        } else {
            // same srs; overlaps -> OK
            if (overlaps(il->second, ir->second)) {
                return true;
            }
        }
    }

    return false;
}

namespace {

void sanityCheck(const TileId &tileId, const Mesh *mesh, const Atlas *atlas
                 , const NodeInfo &nodeInfo)
{
    if (!mesh) {
        if (atlas) {
            LOGTHROW(err1, storage::InconsistentInput)
                << "Tile " << tileId
                << ": atlas cannot exist without mesh.";
        }

        // OK
        return;
    }

    auto imesh(mesh->begin());

    if (atlas) {
        if (atlas->empty()) {
            LOGTHROW(err1, storage::InconsistentInput)
                << "Tile " << tileId << ": empty atlas.";
        }

        if (atlas->size() > mesh->size()) {
            LOGTHROW(err1, storage::InconsistentInput)
                << "Tile " << tileId
                << ": there cannot be more textures than sub-meshes.";
        }

        // check submeshes with texture
        for (std::size_t i(0), e(atlas->size()); i != e; ++i) {
            const auto &sm(*imesh++);
            if (sm.tc.empty()) {
                LOGTHROW(err1, storage::InconsistentInput)
                    << "Tile " << tileId
                    << ": mesh with internal texture without texture "
                    "coordinate.";
            }

            if ((sm.textureMode == SubMesh::TextureMode::external)
                && (sm.etc.empty()))
            {
                LOGTHROW(err1, storage::InconsistentInput)
                    << "Tile " << tileId
                    << ": external texture mode but there are no "
                    "external texture coordinates .";
            }
        }
    }

    // check submeshes without texture
    for (auto emesh(mesh->end()); imesh != emesh; ++imesh) {
        const auto &sm(*imesh);

        if (sm.etc.empty()) {
            LOGTHROW(err1, storage::InconsistentInput)
                << "Tile " << tileId
                << ": mesh without internal texture missing external "
                "texture coordinates.";
        }

        if (sm.textureMode == SubMesh::TextureMode::internal) {
            LOGTHROW(err1, storage::InconsistentInput)
                << "Tile " << tileId
                << ": mesh without internal texture cannot have internal "
                "texture mode.";
        }

        if (!sm.tc.empty()) {
            LOGTHROW(err1, storage::InconsistentInput)
                << "Tile " << tileId
                << ": mesh without internal texture cannot have internal "
                "texture coordinates.";
        }

        if (!nodeInfo.node.externalTexture) {
            LOGTHROW(err1, storage::InconsistentInput)
                << "Tile " << tileId
                << ": reference frame node doesn't allow external texture.";
        }
    }
}

} // namespace

void TileSet::Detail::setTile(const TileId &tileId, const Tile &tile
                              , const NodeInfo *ni)
{
    driver->wannaWrite("set tile");

    LOG(info1) << "Setting content of tile " << tileId << ".";

    auto *mesh(tile.mesh.get());
    auto *atlas(tile.atlas.get());
    auto *navtile(tile.navtile.get());

    // resolve node info
    const NodeInfo nodeInfo(ni ? *ni : NodeInfo(referenceFrame, tileId));

    sanityCheck(tileId, mesh, atlas, nodeInfo);

    MetaNode metanode;

    // set various flags and metadata
    if (mesh) {
        // geometry
        metanode.geometry(true);
        metanode.extents = normalizedExtents(referenceFrame, extents(*mesh));

        // get external textures info configuration
        updateProperties(*mesh);

        metanode.applyTexelSize(true);

        const auto ma(area(*mesh));
        double meshArea(ma.mesh);

        double textureArea(0.0);

        // mesh and texture area -> texelSize
        auto ita(ma.submeshes.begin());

        // internally-textured submeshes
        if (atlas) {
            for (std::size_t i(0), e(atlas->size()); i != e; ++i, ++ita) {
                textureArea += ita->internalTexture * atlas->area(i);
            }

            // set atlas related info
            metanode.internalTextureCount(atlas->size());
        }

        // externally-textured submeshes
        for (auto eta(ma.submeshes.end()); ita != eta; ++ita) {
            textureArea += (ita->externalTexture
                            * registry::BoundLayer::tileArea());
        }

        metanode.texelSize = std::sqrt(meshArea / textureArea);

        // set credits (only when we have mesh)
        metanode.updateCredits(tile.credits);
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

    // update properties
    updateProperties(nodeInfo);
}

void TileSet::Detail::setTile(const TileId &tileId, const TileSource &tile
                              , const NodeInfo *nodeInfo)
{

    // store node
    updateNode(tileId, tile.metanode, tile.watertight);

    // copy data
    if (tile.mesh) {
        copyFile(tile.mesh, driver->output(tileId, TileFile::mesh));
    }

    if (tile.atlas) {
        copyFile(tile.atlas, driver->output(tileId, TileFile::atlas));
    }

    if (tile.navtile) {
        copyFile(tile.navtile, driver->output(tileId, TileFile::navtile));
    }

    // update properties with node info (computed or generated)
    updateProperties
        ((nodeInfo ? *nodeInfo : NodeInfo(referenceFrame, tileId)));
}

void TileSet::Detail::setReferenceTile(const TileId &tileId, uint8_t other)
{
    MetaNode node;
    node.reference(other);
    updateNode(tileId, node, false);
}

Mesh TileSet::Detail::getMesh(const TileId &tileId, const MetaNode *node)
    const
{
    if (!node) {
        LOGTHROW(err2, storage::NoSuchTile)
            << "There is no tile at " << tileId << ".";
    }

    if (!node->geometry()) {
        LOGTHROW(err2, storage::NoSuchTile)
            << "Tile " << tileId << " has no mesh.";
    }

    Mesh mesh;
    load(driver->input(tileId, TileFile::mesh), mesh);
    return mesh;
}

Mesh TileSet::Detail::getMesh(const TileId &tileId) const
{
    return getMesh(tileId, findMetaNode(tileId));
}

void TileSet::Detail::getAtlas(const TileId &tileId, Atlas &atlas
                               , const MetaNode *node) const
{
    if (!node) {
        LOGTHROW(err2, storage::NoSuchTile)
            << "There is no tile at " << tileId << ".";
    }

    if (!node->internalTextureCount()) {
        LOGTHROW(err2, storage::NoSuchTile)
            << "Tile " << tileId << " has no atlas.";
    }

    load(driver->input(tileId, TileFile::atlas), atlas);
}

void TileSet::Detail::getAtlas(const TileId &tileId, Atlas &atlas) const
{
    return getAtlas(tileId, atlas, findMetaNode(tileId));
}

void TileSet::Detail::getNavTile(const TileId &tileId, NavTile &navtile
                                 , const MetaNode *node) const
{
    if (!node) {
        LOGTHROW(err2, storage::NoSuchTile)
            << "There is no tile at " << tileId << ".";
    }

    if (!node->navtile()) {
        LOGTHROW(err2, storage::NoSuchTile)
            << "Tile " << tileId << " has no navtile.";
    }

    load(node->heightRange, driver->input(tileId, TileFile::navtile)
         , navtile);
}

void TileSet::Detail::getNavTile(const TileId &tileId, NavTile &navtile) const
{
    return getNavTile(tileId, navtile, findMetaNode(tileId));
}

TileSource TileSet::Detail::getTileSource(const TileId &tileId) const
{
    const auto *node(findMetaNode(tileId));

    if (!node) {
        LOGTHROW(err2, storage::NoSuchTile)
            << "There is no tile at " << tileId << ".";
    }

    TileSource tile(*node, fullyCovered(tileId));

    if (node->geometry()) {
        tile.mesh = driver->input(tileId, TileFile::mesh);
    }

    if (node->internalTextureCount()) {
        tile.atlas = driver->input(tileId, TileFile::atlas);
    }

    if (node->navtile()) {
        tile.navtile = driver->input(tileId, TileFile::navtile);
    }

    return tile;
}

bool TileSet::Detail::exists(const TileId &tileId) const
{
    // tile exists when it is real (i.e. has mesh and/or atlas)
    if (readOnly) {
        // readonly -> try tileindex
        return tileIndex.real(tileId);
    }

    // rw -> no tileindex generated -> find metatile
    auto ftileNodes(tileNodes.find(tileId));
    if (ftileNodes == tileNodes.end()) { return false; }

    // tile must be real
    return ftileNodes->second.metanode->real();
}

void TileSet::Detail::saveMetadata()
{
    auto maskFromNode([](const TileId &tid, const MetaNode &node)
                      -> std::uint8_t
    {
        (void) tid;
        std::uint8_t m(0);
        if (node.geometry()) { m |= TileIndex::Flag::mesh; }
        if (node.navtile()) { m |= TileIndex::Flag::navtile; }
        if (node.internalTextureCount()) { m |= TileIndex::Flag::atlas; }
        return m;
    });

    driver->wannaWrite("save metadata");

    for (const auto &item : metaTiles) {
        const auto &meta(item.second);
        auto tileId(meta.origin());
        LOG(info1) << "Saving: " << tileId;

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
                tileIndex.setMask(tileId, maskFromNode(tileId, node));
            }
        });

        // mark metatile
        tileIndex.setMask(tileId, TileIndex::Flag::meta);
    }

    saveTileIndex();
}

void update(TileSet::Properties &properties, const TileIndex &tileIndex)
{
    auto stat(tileIndex.statMask(TileIndex::Flag::mesh
                                 | TileIndex::Flag::atlas));
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

    if (metadataChanged) {
        saveMetadata();
        metadataChanged = false;
        // force properties change
        propertiesChanged = true;

        if (!properties.position.valid()
            && !properties.spatialDivisionExtents.empty()) {
            // guess position from spatial division extents
            const auto &item(*properties.spatialDivisionExtents.begin());
            auto &position(properties.position);

            position.type = registry::Position::Type::objective;
            position.orientation = { .0, -90., .0 };
            position.heightMode = registry::Position::HeightMode::fixed;

            math::Point2 p(center(item.second));
            position.position = { p(0), p(1), 1000 };
            position.verticalExtent = 5000;
            position.verticalFov = 90;

            geo::CsConvertor conv(registry::Registry::srs(item.first).srsDef
                                  , registry::Registry::srs
                                  (referenceFrame.model.navigationSrs).srsDef);
            position.position = conv(position.position);
        }
    }

    // update and save config
    if (propertiesChanged) {
        update(properties, tileIndex);
        saveConfig();
        propertiesChanged = false;
    }

    // flush driver
    driver->flush();
}

const Driver& TileSet::driver() const
{
    return *detail().driver;
}

fs::path TileSet::root() const
{
    return detail().driver->root();
}

MapConfig TileSet::mapConfig() const
{
    return detail().mapConfig();
}

ExtraTileSetProperties TileSet::Detail::loadExtraConfig() const
{
    return loadExtraConfig(*driver);
}

ExtraTileSetProperties TileSet::Detail::loadExtraConfig(const Driver &driver)
{
    IStream::pointer is;
    try {
        is = driver.input(File::extraConfig);
    } catch (std::exception) {
        return {};
    }

    return tileset::loadExtraConfig(*is);
}

const TileIndex& TileSet::tileIndex() const
{
    return detail().tileIndex;
}

MapConfig TileSet::mapConfig(const boost::filesystem::path &root)
{
    return Detail::mapConfig(Driver(root));
}

MapConfig TileSet::Detail::mapConfig(const Driver &driver)
{
    return mapConfig(loadConfig(driver), loadExtraConfig(driver));
}

MapConfig TileSet::Detail::mapConfig() const
{
    return mapConfig(properties, loadExtraConfig());
}

MapConfig
TileSet::Detail::mapConfig(const Properties &properties
                           , const ExtraTileSetProperties &extra)
{
    auto referenceFrame(registry::Registry::referenceFrame
                        (properties.referenceFrame));

    MapConfig mapConfig;

    mapConfig.referenceFrame = referenceFrame;
    mapConfig.srs = registry::listSrs(referenceFrame);
    mapConfig.credits = registry::creditsAsDict(properties.credits);
    mapConfig.boundLayers
        = registry::boundLayersAsDict(properties.boundLayers);

    mapConfig.surfaces.emplace_back();
    auto &surface(mapConfig.surfaces.back());
    surface.id = properties.id;
    surface.revision = properties.revision;

    // local path
    surface.root = fs::path();

    if (!properties.lodRange.empty()) {
        surface.lodRange = properties.lodRange;
    }

    if (valid(properties.tileRange)) {
        surface.tileRange = properties.tileRange;
    }

    // apply extra config
    mapConfig.credits.update
        (registry::creditsAsDict(extra.extraCredits));
    mapConfig.boundLayers.update
       (registry::boundLayersAsDict(extra.extraBoundLayers));

    surface.textureLayer = extra.textureLayer;

    mapConfig.position
        = extra.position ? *extra.position : properties.position;

    mapConfig.rois = extra.rois;

    // just one surface in the view
    mapConfig.view.surfaces.push_back(surface.id);

    return mapConfig;
}

void TileSet::Detail::setPosition(const registry::Position &position)
{
    driver->wannaWrite("set position");
    properties.position = position;
    propertiesChanged = true;
}

void TileSet::Detail::addCredits(const registry::IdSet &credits)
{
    driver->wannaWrite("add credits");
    properties.credits.insert(credits.begin(), credits.end());
    propertiesChanged = true;
}

void TileSet::Detail::addBoundLayers(const registry::IdSet &boundLayers)
{
    driver->wannaWrite("add bound layers");
    properties.boundLayers.insert(boundLayers.begin(), boundLayers.end());
    propertiesChanged = true;
}

TileIndex TileSet::sphereOfInfluence(const LodRange &range
                                     , TileIndex::Flag::value_type type)
    const
{
    const auto lr(range.empty() ? detail().tileIndex.lodRange() : range);
    return detail().tileIndex.grow(lr, type);
}

TileIndex TileSet::tileIndex(const LodRange &lodRange) const
{
    return TileIndex(lodRange, &detail().tileIndex);
}

bool TileSet::check(const boost::filesystem::path &root)
{
    try {
        Detail::loadConfig(Driver(root));
    } catch (const storage::Error&) {
        return false;
    }
    return true;
}

bool TileSet::externallyChanged() const
{
    return detail().driver->externallyChanged();
}

storage::Resources TileSet::resources() const
{
    return detail().driver->resources();
}

std::time_t TileSet::lastModified() const
{
    return detail().driver->lastModified();
}

bool TileSet::Detail::fullyCovered(const TileId &tileId) const
{
    return (tileIndex.get(tileId) & TileIndex::Flag::watertight);
}

bool TileSet::canContain(const NodeInfo &nodeInfo) const
{
    const auto &sde(detail().properties.spatialDivisionExtents);
    auto fsde(sde.find(nodeInfo.node.srs));
    if (fsde == sde.end()) { return false; }
    return overlaps(fsde->second, nodeInfo.node.extents);
}

int TileSet::getReference(const TileId &tileId) const
{
    if (!detail().tileIndex.checkMask(tileId, TileIndex::Flag::reference)) {
        return 0;
    }
    return detail().references.get(tileId);
}

void TileSet::paste(const TileSet &srcSet
                    , const boost::optional<LodRange> &lodRange)
{
    const auto &src(srcSet.detail());
    auto &dst(detail());

    const auto reportName(str(boost::format("Pasting <%s> into <%s> ")
                              % src.properties.id
                              % dst.properties.id));

    const auto &sd(*src.driver);
    auto &dd(*dst.driver);

    const utility::Progress::ratio_t reportRatio(1, 100);
    utility::Progress progress(src.tileIndex.count());
    auto report([&]() { (++progress).report(reportRatio, reportName); });

    LOG(info3) << "About to paste " << progress.total() << " tiles.";

    traverse(src.tileIndex, [&](const TileId &tid, QTree::value_type mask)
    {
        // skip
        if (lodRange && !in(*lodRange, tid.lod)) {
            report();
            return;
        }

        const auto *metanode(src.findMetaNode(tid));
        if (!metanode) {
            if (mask & TileIndex::Flag::content) {
                LOG(warn2)
                    << "Cannot find metanode for tile " << tid << "; "
                    << "skipping (flags: " << std::bitset<8>(mask)
                    << ").";
            }
            report();
            return;
        }

        if (mask & TileIndex::Flag::mesh) {
            // copy mesh
            copyFile(sd.input(tid, storage::TileFile::mesh)
                     , dd.output(tid, storage::TileFile::mesh));
        }

        if (mask & TileIndex::Flag::atlas) {
            // copy atlas
            copyFile(sd.input(tid, storage::TileFile::atlas)
                     , dd.output(tid, storage::TileFile::atlas));
        }

        if (mask & TileIndex::Flag::navtile){
            // copy navtile if allowed
            copyFile(sd.input(tid, storage::TileFile::navtile)
                     , dd.output(tid, storage::TileFile::navtile));
        }

        dst.updateNode(tid, *metanode
                       , mask & TileIndex::Flag::watertight);
        LOG(info1) << "Stored tile " << tid << ".";
        report();
    });

    // properties have been changed
    dst.propertiesChanged = true;
}

TileSet concatTileSets(const boost::filesystem::path &path
                       , const std::vector<fs::path> &tilesets
                       , const CloneOptions &createOptions)
{
    if (tilesets.empty()) {
        LOGTHROW(err2, storage::NoSuchTile)
            << "No tilesets to concatenate.";
    }


    auto co(createOptions);
    if (!co.tilesetId()) {
        co.tilesetId(path.filename().string());
    }

    // fill in tilesets in reverse order (first is at the back)
    std::vector<TileSet> tsList;
    {
        std::string rf;
        for (const auto &p : boost::adaptors::reverse(tilesets)) {
            tsList.push_back(openTileSet(p));
            const auto prop(tsList.back().getProperties());
            if (rf.empty()) {
                rf = prop.referenceFrame;
            } else if (rf != prop.referenceFrame) {
                LOGTHROW(err1, vadstena::storage::IncompatibleTileSet)
                    << "Tileset <" << prop.id << "> "
                    "uses different reference frame ("
                    << prop.referenceFrame
                    << ") than other tilesets ("
                    << rf << ").";
            }
        }
    }

    // create new tileset with same properties as the first one (except its id)
    auto properties(tsList.front().getProperties());
    if (co.tilesetId()) {
        properties.id = *co.tilesetId();
    }
    auto dst(createTileSet(path, properties, co.mode()));

    // clone-in all tileset from back (i.e. in the order they have been
    // specified bu user
    for (; !tsList.empty(); tsList.pop_back()) {
        dst.paste(tsList.back(), createOptions.lodRange());
    }

    dst.flush();

    return dst;
}

} } // namespace vadstena::vts
