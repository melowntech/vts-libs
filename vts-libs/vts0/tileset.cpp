/**
 * Copyright (c) 2017 Melown Technologies SE
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * *  Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#include <cstdlib>
#include <memory>
#include <queue>

#include <boost/format.hpp>
#include <boost/utility/in_place_factory.hpp>

#include <opencv2/highgui/highgui.hpp>

#include "dbglog/dbglog.hpp"
#include "utility/binaryio.hpp"
#include "utility/progress.hpp"
#include "utility/gccversion.hpp"

#include "geo/srsdef.hpp"
#include "geometry/binmesh.hpp"

#include "../vts0.hpp"
#include "config.hpp"
#include "tileindex.hpp"
#include "io.hpp"
#include "tileindex-io.hpp"
#include "tileopext.hpp"
#include "tileset-detail.hpp"
#include "tileset-advanced.hpp"
#include "metatile.hpp"
#include "metatileop.hpp"
#include "merge.hpp"
#include "tileset/dump.hpp"
#include "driver/tilardriver.hpp"

namespace vtslibs { namespace vts0 {

namespace fs = boost::filesystem;

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
    auto mesh(geometry::loadBinaryMesh(is->get()));
    is->close();
    return mesh;
}

void saveMesh(const OStream::pointer &os, const Mesh &mesh)
{
    geometry::writeBinaryMesh(os->get(), mesh);
    os->close();
}

} // namespace

struct TileSet::Factory
{
    static TileSet::pointer create(const fs::path &path
                                   , const CreateProperties &properties
                                   , CreateMode mode
                                   , bool cloned = false)
    {
        auto driver(std::make_shared<TilarDriver>
                    (path, mode
                     , Driver::CreateProperties
                     (properties.staticProperties.props, cloned)));
        return TileSet::pointer(new TileSet(driver, properties));
    }

    static TileSet::pointer open(const fs::path &path, OpenMode mode)
    {
        auto driver(std::make_shared<TilarDriver>(path, mode));
        return TileSet::pointer(new TileSet(driver));
    }

    static void clone(const TileSet::pointer &src
                      , const TileSet::pointer &dst)
    {
        dst->detail().clone(src->detail());
    }

    static void clone(const TileSet::pointer &src
                      , const TileSet::pointer &dst
                      , const CloneOptions::Filter &filter)
    {
        if (!filter) {
            return clone(src, dst);
        }

        dst->detail().clone(src->detail(), filter);
    }

    static void clone(const TileSet::pointer &src
                      , const TileSet::pointer &dst
                      , const CloneOptions::Filter &filter
                      , const StaticProperties::Wrapper &staticProperties
                      , const SettableProperties::Wrapper &settableProperties)
    {
        {
            auto &sdet(src->detail());
            auto &ddet(dst->detail());

            // copy in source properties
            ddet.properties = sdet.properties;

            // merge in requested changes
            ddet.properties.staticProperties().merge(staticProperties);
            ddet.properties.settableProperties().merge(settableProperties);

            // mark as changed
            ddet.propertiesChanged = true;
        }

        if (!filter) {
            return clone(src, dst);
        }

        dst->detail().clone(src->detail(), filter);
    }
};

TileSet::pointer createTileSet(const fs::path &path
                               , const CreateProperties &properties
                               , CreateMode mode)
{
    return TileSet::Factory::create(path, properties, mode);
}

TileSet::pointer openTileSet(const fs::path &path, OpenMode mode)
{
    return TileSet::Factory::open(path, mode);
}

TileSet::pointer cloneTileSet(const fs::path &path
                              , const fs::path &srcPath
                              , const CloneOptions &options)
{
    return cloneTileSet(path, openTileSet(srcPath, OpenMode::readOnly)
                        , options);
}

TileSet::pointer cloneTileSet(const fs::path &path
                              , const TileSet::pointer &src
                              , const CloneOptions &options)
{
    // merge existing properties with new properties
    CreateProperties properties(src->getProperties());
    properties.staticProperties.props.merge(options.staticProperties);
    properties.settableProperties.props.merge(options.settableProperties);

    // create tileset and clone
    auto dst(TileSet::Factory::create
             (path, properties, options.createMode, true));
    TileSet::Factory::clone(src, dst, options.getFilter(*src));
    return dst;
}

TileSet::pointer cloneTileSet(const TileSet::pointer &dst
                              , const TileSet::pointer &src
                              , const CloneOptions &options)
{
    // make sure dst is flushed
    dst->flush();
    if (!dst->empty() && (options.createMode != CreateMode::overwrite)) {
        LOGTHROW(err2, storage::TileSetNotEmpty)
            << "Tile set <" << dst->getProperties().id
            << "> is not empty.";
    }

    TileSet::Factory::clone(src, dst, options.getFilter(*src)
                            , options.staticProperties
                            , options.settableProperties);
    return dst;
}

TileSet::Detail::Detail(const Driver::pointer &driver)
    : driver(driver), propertiesChanged(false)
    , metadataChanged(false)
    , tx(false)
{
    loadConfig();
    // load tile index only if there are any tiles
    if (properties.hasData) {
        loadTileIndex();
    } else {
        tileIndex = {};
    }
}

TileSet::Detail::Detail(const Driver::pointer &driver
                        , const CreateProperties &properties)
    : driver(driver), propertiesChanged(false)
    , lodRange(LodRange::emptyRange())
    , metadataChanged(false)
    , tx(false)
{
    const auto &sp(properties.staticProperties.props);
    if (sp.id.empty()) {
        LOGTHROW(err2, storage::FormatError)
            << "Cannot create tile set without valid id.";
    }

    if (sp.metaLevels.delta <= 0) {
        LOGTHROW(err2, storage::FormatError)
            << "Tile set must have positive metaLevels.delta.";
    }

    // build initial properties
    auto &p(this->properties);

    // initialize create properties
    static_cast<StaticProperties&>(p)
        .merge(properties.staticProperties);
    static_cast<SettableProperties&>(p)
        .merge(properties.settableProperties);

    // force save of properties
    p.driver = driver->properties();

    // set templates
    p.meshTemplate = "{lod}-{x}-{y}.bin";
    p.textureTemplate = "{lod}-{x}-{y}.jpg";
    p.metaTemplate = "{lod}-{x}-{y}.meta";

    // tile index must be properly initialized
    tileIndex = {};

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

void TileSet::Detail::loadConfig()
{
    try {
        // load config
        auto f(driver->input(File::config));
        const auto p(vts0::loadConfig(*f));
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
        vts0::saveConfig(*f, properties);
        f->close();
    } catch (const std::exception &e) {
        LOGTHROW(err2, storage::Error)
            << "Unable to write config: <" << e.what() << ">.";
    }

    // done; remember saved properties and go on
    savedProperties = properties;
    propertiesChanged = false;
}

void TileSet::Detail::saveMetadata()
{
    driver->wannaWrite("save metadata");

    // create tile index
    TileIndex ti(lodRange);
    LOG(info1) << "New tile index:\n" << ti;

    // create metatile index
    TileIndex mi({0, lodRange.max});
    LOG(info1) << "New metatile index:\n" << mi;

    // well, dump metatiles now
    saveMetatiles(ti, mi);

    // save index
    try {
        auto f(driver->output(File::tileIndex));
        ti.save(*f);
        mi.save(*f);
        f->close();
    } catch (const std::exception &e) {
        LOGTHROW(err2, storage::Error)
            << "Unable to write tile index: " << e.what() << ".";
    }

    // cool, we have new tile/meta index
    tileIndex = ti;
    metaIndex = mi;

    if (metadata.empty()) {
        // no tile, we should invalidate tileset
        properties.hasData = false;
        propertiesChanged = true;
    } else {
        properties.hasData = true;
        propertiesChanged = true;
    }

    // saved => no change
    metadataChanged = false;
}

void TileSet::Detail::loadTileIndex()
{
    try {
        tileIndex = {};
        metaIndex = {};
        auto f(driver->input(File::tileIndex));
        tileIndex.load(*f);
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
        metadataChanged = true;
        return metanode;
    }

    // insert new node
    if (!old) {
        metadata.insert(Metadata::value_type(tileId, metanode));
    }

    // now there surely is one
    auto newNode(*findMetaNode(tileId));

    // update extents/lod-range
    if (lodRange.empty()) {
        // initial lod range
        lodRange.min = lodRange.max = tileId.lod;
    } else {
        // add tile

        // update lod range
        if (tileId.lod < lodRange.min) {
            lodRange.min = tileId.lod;
        } else if (tileId.lod > lodRange.max) {
            lodRange.max = tileId.lod;
        }
    }

    // layout updated -> update zboxes up the tree
    updateTree(tileId, newNode);

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

void loadMetatileFromFile(const IStream::pointer &f, Metadata &metadata
                           , const TileId &tileId
                           , const MetaNodeNotify &notify = MetaNodeNotify())
{
    vts0::loadMetatile
        (*f, tileId, [&metadata]
         (const TileId &tileId, const MetaNode &node, std::uint8_t)
         {
             metadata.insert(Metadata::value_type(tileId, node));
         }
         , notify);
}

void TileSet::Detail
::loadMetatileFromFile(Metadata &metadata, const TileId &tileId
                       , const MetaNodeNotify &notify)
    const
{
    auto f(driver->input(tileId, TileFile::meta));
    vts0::loadMetatileFromFile
        (f, metadata, tileId, notify);
    f->close();
}

MetaNode* TileSet::Detail::loadMetatile(const TileId &tileId)
    const
{
    // no tile (or metatile) cannot be in an lod below lowest level in the tile
    // set)
    if (tileId.lod > lodRange.max) {
        return nullptr;
    }

    // sanity check
    if (!savedProperties.hasData) {
        // no data on disk
        return nullptr;
    }

    auto metaId(findMetatile(savedProperties.metaLevels, tileId));

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
        LOGTHROW(err2, storage::NoSuchTile)
            << "There is no tile at " << tileId << ".";
    }

    // assign new metadata
    static_cast<TileMetadata&>(*metanode) = metadata;

    metadataChanged = true;
}

void TileSet::Detail::check(const TileId &tileId) const
{
    (void) tileId;
}

void TileSet::Detail::checkTx(const std::string &action) const
{
    if (!tx) {
        LOGTHROW(err2, storage::PendingTransaction)
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
    for (const auto &childId : children(tileId)) {
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
    for (const auto &childId : children(tileId)) {
        if (auto *node = findMetaNode(childId)) {
            metanode.zmin = std::min(metanode.zmin, node->zmin);
            metanode.zmax = std::max(metanode.zmax, node->zmax);
            minGsd = std::min(minGsd, node->gsd);
            minGsdSet = true;
        }
    }

    if (minGsdSet) {
        metanode.gsd = minGsd;
    }

    if (!tileId.lod) {
        // reached root
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

void TileSet::Detail::flush()
{
    if (driver->readOnly()) { return; }
    LOG(info3) << "Tile set <" << properties.id << ">: flushing.";

    // force metadata save
    if (metadataChanged) {
        saveMetadata();

        // metadata were changed but we have to ensure that config file's last
        // modification timestamp changes to tell outer world (e.g. libhttp)
        // that something has been changed
        propertiesChanged = true;
    }

    // force config save
    if (propertiesChanged) {
        saveConfig();
    }

    // flush driver
    driver->flush();
    LOG(info3) << "Tile set <" << properties.id << ">: flushed.";
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

    vts0::saveMetatile({}, properties.metaLevels
                      , Saver(*this, tileIndex, metaIndex));
}

Tile TileSet::Detail::getTile(const TileId &tileId) const
{
    check(tileId);

    auto md(findMetaNode(tileId));
    if (!md || !md->exists()) {
        LOGTHROW(err2, storage::NoSuchTile)
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
    calcParams(metanode, mesh, { atlas.cols, atlas.rows }, pixelSize);

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
        LOGTHROW(err2, storage::PendingTransaction)
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
        LOGTHROW(err2, storage::PendingTransaction)
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
        LOGTHROW(err2, storage::PendingTransaction)
            << "There is no active transaction to roll back.";

    }

    // TODO: what to do if anything throws?

    driver->rollback();

    // re-read backing store state
    loadConfig();

    // destroy tile index
    tileIndex = {};
    metadata = {};
    loadedMetatiles = {};
    metadataChanged = false;
    propertiesChanged = false;

    if (savedProperties.hasData) {
        // load tile index only if there are any tiles
        loadTileIndex();
    }

    // no pending tx
    tx = false;
}

void TileSet::Detail::watch(utility::Runnable *runnable)
{
    driver->watch(runnable);
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

MetaNode TileSet::getMetadata(const TileId &tileId) const
{
    detail().checkValidity();
    auto md(detail().findMetaNode(tileId));
    if (!md || !md->exists()) {
        LOGTHROW(err2, storage::NoSuchTile)
            << "There is no tile at " << tileId << ".";
    }

    return *md;
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

void TileSet::watch(utility::Runnable *runnable)
{
    detail().checkValidity();
    detail().watch(runnable);
}

bool TileSet::inTx() const
{
    return detail().tx;
}

bool TileSet::empty() const
{
    // has tiles?
    return !detail().properties.hasData;
}

void TileSet::drop()
{
    detail().driver->drop();
    // make invalid!
    detail().driver.reset();
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

FileStat TileSet::AdvancedApi::stat(File type) const
{
    const auto &detail(tileSet_->detail());
    detail.checkValidity();
    return detail.driver->stat(type);
}

FileStat TileSet::AdvancedApi::stat(const TileId tileId, TileFile type)
    const
{
    const auto &detail(tileSet_->detail());
    detail.checkValidity();
    return detail.driver->stat(tileId, type);
}


MetaNode TileSet::AdvancedApi::setMetaNode(const TileId &tileId
                                           , const MetaNode& metanode)
{
    auto &detail(tileSet_->detail());
    detail.checkValidity();
    return detail.setMetaNode(tileId, metanode);
}

void TileSet::AdvancedApi::regenerateTileIndex()
{
    // FIXME: not implemented so far

    // TODO: ensure that there are no pending changes
    auto &detail(tileSet_->detail());
    detail.checkValidity();

    if (!detail.savedProperties.hasData) {
        // TODO: save empty index if nothing there
        return;
    }

    Metadata metadata;

    std::queue<TileId> subtrees({0, 0, 0});

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

        for (const auto &childId : children(tileId)) {
            forceMetadata(childId, metadata, mask);
        }
    };
}

void TileSet::AdvancedApi::forceMetadata( const TileMetadata &metadata
                                        , const TileMetadata::MaskType mask)
{
    (void) mask;
    auto &detail(tileSet_->detail());

    forceMetadata({}, metadata, mask);

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

    if (oProps.extents != props.extents) {
        LOG(warn2)
            << "Tile set <" << props.id
            << ">: set <" << oProps.id
            << "> has incompatible root extents.";
        return false;
    }

    return true;
}

TileSet::Statistics TileSet::Detail::stat() const
{
    if (metadataChanged) {
        LOGTHROW(warn2, storage::TileSetNotFlushed)
            << "Tileset <" << properties.id << " is not flushed, "
            << "unable to get proper statistics.";
    }
    return {
        tileIndex.count()
        , metaIndex.count()
    };
}

LodRange TileSet::lodRange() const {return detail().lodRange; }

TileSet::Statistics TileSet::stat() const { return detail().stat(); }

void pasteTileSets(const TileSet::pointer &dst
                   , const TileSet::list &src
                   , utility::Runnable *runnable)
{
    try {
        // paste tiles
        dst->watch(runnable);
        dst->paste(src);

        // creates and immediately commits a transaction -> generates metadata
        // in tx that is flushed and commited
        if (!dst->inTx()) {
            // no pending transaction -> create one :)
            dst->begin(runnable);
            dst->commit();
        } else {
            // just flush
            dst->flush();
        }
    } catch (const std::exception &e) {
        LOG(warn3)
            << "Operation being rolled back due to an error: <"
            << e.what() << ">.";
        if (dst->inTx()) {
            dst->rollback();
        }
    }

}

void TileSet::Detail::clone(const Detail &src)
{
    // update properties
    properties.driver = driver->properties();
    properties.extents = src.properties.extents;
    propertiesChanged = true;

    const auto &sd(*src.driver);
    auto &dd(*driver);

    // copy single files
    for (auto type : { File::tileIndex  }) {
        copyFile(sd.input(type), dd.output(type));
    }

    const utility::Progress::ratio_t reportRatio(1, 100);
    const auto name(str(boost::format("Cloning <%s> ")
                        % src.properties.id));
    utility::Progress progress(src.tileIndex.count()
                               + src.metaIndex.count());

    // copy tiles
    traverseTiles(src.tileIndex, [&](const TileId &tileId)
    {
        for (auto type : { TileFile::mesh, TileFile::atlas }) {
            copyFile(sd.input(tileId, type), dd.output(tileId, type));
        }
        (++progress).report(reportRatio, name);

        // mark that we have data
        properties.hasData = true;
    });

    // copy metatiles
    const auto canCopy(src.properties.metaLevels == properties.metaLevels);
    traverseTiles(src.metaIndex, [&](const TileId &metaId)
    {
        if (canCopy) {
            // same meta levels -> just copy file
            copyFile(sd.input(metaId, TileFile::meta)
                     , dd.output(metaId, TileFile::meta));
        } else {
            // must load into memory, saved in flush
            auto f(sd.input(metaId, TileFile::meta));
            vts0::loadMetatileFromFile(f, metadata, metaId);
            f->close();
        }
        (++progress).report(reportRatio, name);
    });

    if (!canCopy) {
        // copy LOD range to have save tile index
        lodRange = src.tileIndex.lodRange();

        // tell flush we have to dump metadata to storage
        metadataChanged = true;
    }

    flush();

    // load index if we have anything
    if (savedProperties.hasData) {
        // load tile index only if there are any data
        loadTileIndex();
    }
}

void TileSet::Detail::clone(const Detail &src
                            , const CloneOptions::Filter &filter)
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
        if (!filter(tileId.lod, extents(properties, tileId))) {
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
    if (savedProperties.hasData) {
        // load tile index only if there are any data
        loadTileIndex();
    }
}

CloneOptions::Filter CloneOptions::getFilter(TileSet &tileSet) const
{
    if (filter) { return filter; }
    if (filterFactory) { return filterFactory(tileSet); }
    return {};
}

} } // namespace vtslibs::vts0
