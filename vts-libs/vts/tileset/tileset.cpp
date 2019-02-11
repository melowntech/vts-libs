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
#include <boost/format.hpp>
#include <boost/filesystem.hpp>
#include <boost/range/adaptor/reversed.hpp>

#include "utility/progress.hpp"
#include "utility/path.hpp"
#include "utility/openmp.hpp"

#include "../../vts.hpp"
#include "../tileset.hpp"
#include "../tileindex-io.hpp"
#include "../io.hpp"
#include "../csconvertor.hpp"
#include "detail.hpp"
#include "driver.hpp"
#include "config.hpp"

namespace fs = boost::filesystem;

namespace vs = vtslibs::storage;

namespace vtslibs { namespace vts {

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

MeshMask TileSet::getMeshMask(const TileId &tileId, bool generate) const
{
    return detail().getMeshMask(tileId, generate);
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

void TileSet::setNavTile(const TileId &tileId, const NavTile &navtile)
{
    detail().setNavTile(tileId, navtile);
}

void TileSet::setSurrogateValue(const TileId &tileId, float value)
{
    detail().setSurrogateValue(tileId, value);
}

MetaNode TileSet::getMetaNode(const TileId &tileId) const
{
    auto node(detail().findNode(tileId));
    if (!node) {
        LOGTHROW(err2, storage::NoSuchTile)
            << "There is no tile at " << tileId << ".";
    }
    return *node.metanode;
}

const MetaNode* TileSet::getMetaNode(const TileId &tileId
                                     , const std::nothrow_t&)
    const
{
    auto node(detail().findNode(tileId));
    return node ? node.metanode : nullptr;
}

MetaTile TileSet::getMetaTile(const TileId &metaId) const
{
    auto mt(detail().findMetaTile(metaId));
    if (!mt) {
        LOGTHROW(err2, storage::NoSuchTile)
            << "There is no metatile at " << metaId << ".";
    }
    return *mt;
}

int TileSet::getMetaTileVersion(const TileId &metaId) const
{
    return detail().getMetaTileVersion(metaId);
}

bool TileSet::exists(const TileId &tileId) const
{
    return detail().exists(tileId);
}

int TileSet::sourceReference(const TileId &tileId) const
{
    const auto &ti(detail().tileIndex);
    const auto flags(ti.get(tileId));

    if (!TileIndex::Flag::isReal(flags)) { return 0; }

    if (const auto sr = TileIndex::Flag::getReference(flags)) {
        return sr;
    }

    return 1;
}

bool TileSet::fullyCovered(const TileId &tileId) const
{
    return detail().fullyCovered(tileId);
}

void TileSet::flush()
{
    detail().flush();
}

void TileSet::emptyCache() const
{
    detail().emptyCache();
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

TileRange TileSet::tileRange(Lod lod) const
{
    return detail().tileIndex.tileRange(lod, TileIndex::Flag::real);
}

namespace {

driver::PlainOptions plainOptions(const TileSetProperties &properties)
{
    // find reference frame
    const auto &referenceFrame(registry::system.referenceFrames
                               (properties.referenceFrame));

    // force aggregation to 5, set meta-unused-bits to meta-binary-order
    return driver::PlainOptions(5, referenceFrame.metaBinaryOrder);
}

GeomExtents geomExtents(const NodeInfo &ni, const Mesh &mesh)
{
    // re-compute geom extents
    return geomExtents
        (CsConvertor(ni.referenceFrame().model.physicalSrs, ni.srs())
         , mesh);
}

fs::path removeTrailingSlash(fs::path path)
{
    if (path.filename() != ".") { return path; }

    // remove trailing slash
    fs::path tmp;
    for (auto i(path.begin()), e(std::prev(path.end())); i != e; ++i) {
        tmp /= *i;
    }
    return tmp;
}

} // namespace

struct TileSet::Factory
{
    static TileSet create(const fs::path &path
                          , const TileSet::Properties &properties
                          , const CloneOptions &co)
    {
        auto driver(Driver::create(path, properties.driverOptions, co));
        return TileSet(driver, properties);
    }

    /** Open using created driver.
     */
    static TileSet open(const Driver::pointer &driver)
    {
        return TileSet(driver);
    }

    static TileSet open(const fs::path &path, const OpenOptions &openOptions)
    {
        auto driver(Driver::open(path, openOptions));
        return TileSet(driver);
    }

    static void clone(const std::string &reportName
                      , const Detail &src, Detail &dst)
    {
        const auto &sd(*src.driver);
        auto &dd(*dst.driver);

        copyFile(sd.input(storage::File::tileIndex)
                 , dd.output(storage::File::tileIndex));

        // clone registry
        if (auto registry = sd.input(File::registry, NullWhenNotFound)) {
            copyFile(registry, dd.output(File::registry));
        }

        auto metaIndex(src.tsi.deriveMetaIndex());

        const utility::Progress::ratio_t reportRatio(1, 100);
        utility::Progress progress(src.tileIndex.count()
                                   + metaIndex.count());
        auto report([&]() { (++progress).report(reportRatio, reportName); });

        traverse(src.tileIndex, [&](const TileId &tid, QTree::value_type mask)
        {
            LOG(debug) << "Storing tile " << tid << ".";

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

            if (mask & TileIndex::Flag::navtile) {
                // copy navtile if allowed
                copyFile(sd.input(tid, storage::TileFile::navtile)
                         , dd.output(tid, storage::TileFile::navtile));
            }

            LOG(info1) << "Stored tile " << tid << ".";
            report();
        });

        // clone metatiles
        auto mbo(src.referenceFrame.metaBinaryOrder);
        traverse(metaIndex, [&](TileId tid, QTree::value_type)
        {
            // expand shrinked metatile identifiers
            tid.x <<= mbo;
            tid.y <<= mbo;

            // copy metatile
            copyFile(sd.input(tid, storage::TileFile::meta)
                     , dd.output(tid, storage::TileFile::meta));

            LOG(info1) << "Stored metatile " << tid << ".";
            report();
        });

        // load copied tile index
        dst.loadTileIndex();

        // properties have been changed
        dst.propertiesChanged = true;
    }

    /** Copy file under lock.
     */
    static void copyFileLocked(const Driver &sd, Driver &dd
                               , const TileId &tileId
                               , vs::TileFile type)
    {
        vs::IStream::pointer is(sd.input(tileId, type));
        UTILITY_OMP(critical(clone_dd))
            copyFile(is, dd.output(tileId, type));
    }

    static void reencode(const TileId &tileId, const NodeInfo &ni
                         , const Driver &sd, Driver &dd
                         , bool hasMesh, bool hasAtlas
                         , CloneOptions::EncodeFlag::value_type eflags
                         , MetaNode &metanode
                         , int textureQuality)
    {
        Mesh mesh;
        RawAtlas atlas;

        if (hasMesh) {
            mesh = loadMesh(sd.input(tileId, storage::TileFile::mesh));
        }

        if (hasAtlas) {
            auto is(sd.input(tileId, storage::TileFile::atlas));
            atlas.deserialize(is->get(), is->name());
        }

        if (hasMesh) {
            if (eflags & CloneOptions::EncodeFlag::mesh) {
                UTILITY_OMP(critical(clone_dd))
                {
                    // reencode mesh
                    auto os(dd.output(tileId, storage::TileFile::mesh));
                    saveMesh(os, mesh, &atlas);
                    os->close();
                }
            } else {
                // just copy file
                copyFileLocked(sd, dd, tileId, storage::TileFile::mesh);
            }

            if ((eflags & CloneOptions::EncodeFlag::meta)
                && vts::empty(metanode.geomExtents))
            {
                // reencoding metanodes and no geometric extents available ->
                // recompute
                metanode.geomExtents = geomExtents(ni, mesh);
                // use average height as a surrogate
                metanode.geomExtents.makeAverageSurrogate();
            }
        }

        if (hasAtlas) {
            if (hasMesh && (eflags & CloneOptions::EncodeFlag::inpaint)) {
                // inpaint
                const auto out(inpaint(atlas, mesh, textureQuality));

                UTILITY_OMP(critical(clone_dd))
                {
                    auto os(dd.output(tileId, storage::TileFile::atlas));
                    out->serialize(os->get());
                    os->close();
                }
            } else {
                // just copy file
                copyFileLocked(sd, dd, tileId, storage::TileFile::atlas);
            }
        }
    }

    static void clone(const std::string &reportName
                      , const Detail *src, Detail *dst
                      , const CloneOptions *cloneOptions)
    {
        // simple case? use fully optimized version
        if (!(cloneOptions->lodRange()
              || cloneOptions->metaNodeManipulator()
              || cloneOptions->encodeFlags()))
        {
            return clone(reportName, *src, *dst);
        }

        const auto *sd(&*src->driver);
        auto *dd(&*dst->driver);
        auto lodRange(cloneOptions->lodRange()
                      ? *cloneOptions->lodRange() : src->lodRange);
        auto mnm_(cloneOptions->metaNodeManipulator());
        auto mnm(&mnm_);

        // clone registry
        if (auto registry = sd->input(File::registry, NullWhenNotFound)) {
            copyFile(registry, dd->output(File::registry));
        }

        const utility::Progress::ratio_t reportRatio(1, 100);
        utility::ts::Progress progress(reportName, src->tileIndex.count()
                                       , reportRatio);
        auto report([&]() { ++progress; });

        auto eflags(cloneOptions->encodeFlags());

        if (eflags) {
            // renencoding, update revision if needed
            if (dst->properties.revision <= src->properties.revision) {
                // destination revision is not newer than source revision, fix
                dst->properties.revision = src->properties.revision + 1;
            }
        }

        UTILITY_OMP(parallel)
        UTILITY_OMP(single)
        traverse(src->tileIndex, [=](TileId tid, QTree::value_type mask)
        {
            // skip out-of range
            if (!in(lodRange, tid.lod)) {
                report();
                return;
            }

            const MetaNode *metanode;
            UTILITY_OMP(critical(clone_sd))
                metanode = src->findMetaNode(tid);

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

            UTILITY_OMP(task)
            {
                bool mesh(mask & TileIndex::Flag::mesh);
                bool atlas(mask & TileIndex::Flag::atlas);

                // optional copy of metanode
                boost::optional<MetaNode> mn;

                auto copyMetanode([&]() -> MetaNode&
                {
                    if (!mn) { mn = *metanode; }
                    return *mn;
                });

                // get reference to metanode
                auto useMetanode([&]() -> const MetaNode&
                {
                    return mn ? *mn : *metanode;
                });

                if (eflags) {
                    reencode(tid, NodeInfo(src->referenceFrame, tid)
                             , *sd, *dd, mesh, atlas, eflags, copyMetanode()
                             , cloneOptions->textureQuality());
                } else {
                    if (mesh) {
                        // copy mesh
                        copyFileLocked(*sd, *dd, tid, storage::TileFile::mesh);
                    }

                    if (atlas) {
                        // copy atlas
                        copyFileLocked(*sd, *dd, tid
                                       , storage::TileFile::atlas);
                    }
                }

                if (mask & TileIndex::Flag::navtile) {
                    // copy navtile if allowed
                    copyFileLocked(*sd, *dd, tid, storage::TileFile::navtile);
                }

                UTILITY_OMP(critical(clone_dd))
                {
                    if (*mnm) {
                        // filter metanode
                        dst->updateNode(tid, (*mnm)(useMetanode())
                                        , (mask & TileIndex::Flag::nonmeta));
                    } else {
                        // pass metanode as-is
                        dst->updateNode(tid, useMetanode()
                                        , (mask & TileIndex::Flag::nonmeta));
                    }

                    if (TileIndex::Flag::isInfluenced(mask)) {
                        // mark as influenced tile

                        // TODO: mark only if we have copied content tile from
                        // above this tile as well
                        dst->markInfluencedTile(tid);
                    }
                }

                LOG(info1) << "Stored tile " << tid << ".";
                report();
            }
        });

        // properties have been changed
        dst->propertiesChanged = true;
    }

    static TileSet clone(const boost::filesystem::path &path
                         , const TileSet &src
                         , const CloneOptions &cloneOptions)
    {
        auto properties(src.detail().properties);
        if (cloneOptions.tilesetId()) {
            properties.id = *cloneOptions.tilesetId();
        }

        if (cloneOptions.sameType()) {
            if (auto driver = src.driver().clone(path, cloneOptions)) {
                return open(driver);
            }
        }

        // assisted cloning must be performed

        // regular type
        properties.driverOptions = plainOptions(src.getProperties());

        auto dst(TileSet::Factory::create(path, properties, cloneOptions));

        const auto reportName(str(boost::format("Cloning <%s> ")
                                  % src.detail().properties.id));
        clone(reportName, &src.detail(), &dst.detail(), &cloneOptions);

        // and flush
        dst.flush();

        return dst;
    }

    static void reencode(const boost::filesystem::path &root
                         , const ReencodeOptions &options)
    {
        // absolutize source path and remove offending trailing slash
        const auto srcPath(removeTrailingSlash(fs::absolute(root)));

        // build dst path (srcPath + .tag)
        const auto dstPath(utility::addExtension(srcPath, "." + options.tag));

        if (options.cleanup) {
            // remove temporary dataset
            fs::remove_all(dstPath);
            return;
        }

        // clone tileset from srcPath to dstPath
        clone(dstPath, open(srcPath, {})
              , CloneOptions()
              .mode(CreateMode::overwrite)
              .encodeFlags(options.encodeFlags)
              );

        // swap srcPath and dstPath
        const auto tmp(utility::addExtension(srcPath, ".swap"));
        fs::rename(srcPath, tmp);
        fs::rename(dstPath, srcPath);
        fs::rename(tmp, dstPath);
    }
};

TileSet createTileSet(const boost::filesystem::path &path
                      , const TileSetProperties &properties
                      , CreateMode mode)
{
    TileSet::Properties tsprop(properties);
    tsprop.driverOptions = plainOptions(properties);

    return TileSet::Factory::create
        (path, tsprop
         , CloneOptions().mode(mode).tilesetId(properties.id));
}

TileSet createTileSet(const boost::filesystem::path &path
                      , const TileSet::Properties &properties
                      , CreateMode mode)
{
    return TileSet::Factory::create
        (path, properties
         , CloneOptions().mode(mode).tilesetId(properties.id));
}

TileSet openTileSet(const boost::filesystem::path &path
                    , const OpenOptions &openOptions)
{
    return TileSet::Factory::open(path, openOptions);
}

TileSet cloneTileSet(const boost::filesystem::path &path, const TileSet &src
                     , const CloneOptions &cloneOptions)
{
    return TileSet::Factory::clone(path, src, cloneOptions);
}

void reencodeTileSet(const boost::filesystem::path &root
                     , const ReencodeOptions &options)
{
    return TileSet::Factory::reencode(root, options);
}

/** Core implementation.
 */
TileSet aggregateTileSets(const boost::filesystem::path &path
                          , const boost::filesystem::path &storagePath
                          , const CloneOptions &co
                          , const TilesetIdSet &tilesets)
{
    driver::AggregatedOptions dopts;
    dopts.storagePath = ((co.createFlags() & AggregateFlags::dontAbsolutize)
                         ? storagePath : fs::absolute(storagePath));
    dopts.tilesets = tilesets;
    dopts.surfaceReferences = (co.createFlags()
                               & AggregateFlags::sourceReferencesInMetatiles);
    if (co.lodRange()) { dopts.staticMetaRange = *co.lodRange(); }

    // TODO: use first non-empty path element
    CloneOptions useCo(co);
    if (!useCo.tilesetId()) { useCo.tilesetId(path.filename().string()); }

    auto driver(Driver::create(path, dopts, useCo));
    return TileSet::Factory::open(driver);
}

/** Core implementation.
 */
TileSet aggregateTileSets(const Storage &storage
                          , const CloneOptions &co
                          , const TilesetIdSet &tilesets)
{
    driver::AggregatedOptions dopts;
    dopts.storagePath = ((co.createFlags() & AggregateFlags::dontAbsolutize)
                         ? storage.path()
                         : fs::absolute(storage.path()));
    dopts.tilesets = tilesets;
    dopts.surfaceReferences = (co.createFlags()
                               & AggregateFlags::sourceReferencesInMetatiles);
    if (co.lodRange()) { dopts.staticMetaRange = *co.lodRange(); }

    auto driver(Driver::create(dopts, co));
    return TileSet::Factory::open(driver);
}

TileSet createRemoteTileSet(const boost::filesystem::path &path
                            , const std::string &url
                            , const CloneOptions &createOptions)

{
    driver::RemoteOptions dopts;
    dopts.url = url;

    auto driver(Driver::create(path, dopts, createOptions));
    return TileSet::Factory::open(driver);
}

TileSet createLocalTileSet(const boost::filesystem::path &path
                           , const boost::filesystem::path &localPath
                           , const CloneOptions &createOptions)

{
    driver::LocalOptions dopts;
    dopts.path = localPath;

    auto driver(Driver::create(path, dopts, createOptions));
    return TileSet::Factory::open(driver);
}

TileSet::Detail::Detail(const Driver::pointer &driver)
    : driverTsi_(driver->getTileIndex())
    , readOnly(true), driver(driver)
    , propertiesChanged(false), metadataChanged(false)
    , metaTiles(MetaCache::create(driver))
    , tsi(driverTsi_ ? *driverTsi_ : tsi_)
    , tileIndex(tsi.tileIndex)
{
    loadConfig();
    referenceFrame = registry::system.referenceFrames
        (properties.referenceFrame);

    if (!driverTsi_) {
        loadTileIndex();
    } else {
        lodRange = tileIndex.lodRange();
    }
}

TileSet::Detail::Detail(const Driver::pointer &driver
                        , const TileSet::Properties &properties)
    : referenceFrame(registry::system.referenceFrames
                     (properties.referenceFrame))
    , tsi_(referenceFrame.metaBinaryOrder)
    , driverTsi_()
    , readOnly(false), driver(driver)
    , propertiesChanged(false), metadataChanged(false)
    , metaTiles(MetaCache::create(driver))
    , tsi(tsi_)
    , tileIndex(tsi.tileIndex)
    , lodRange(LodRange::emptyRange())
{

    if (properties.id.empty()) {
        LOGTHROW(err2, storage::FormatError)
            << "Cannot create tile set without valid id.";
    }

    // build initial properties
    this->properties = properties;
    this->properties.driverOptions = driver->options();

    // if there was some old file here, update revision
    if (auto oldRevision = driver->oldRevision()) {
        this->properties.revision = *oldRevision + 1;
    }

    // save config and (empty) tile index and reference
    saveConfig();
    saveTileIndex();

    // copy read-only flag from driver after everything has been done
    readOnly = driver->readOnly();
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
    properties = tileset::loadConfig(*driver);
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
    // initialize tileset index with proper settings
    tsi_ = { referenceFrame.metaBinaryOrder };
    tileset::loadTileSetIndex(tsi_, *driver);

    // new extents
    lodRange = tileIndex.lodRange();
    LOG(debug) << "Loaded tile index: " << tsi_.tileIndex;
}

void TileSet::Detail::saveTileIndex()
{
    tileset::saveTileSetIndex(tsi_, *driver);
}

void TileSet::Detail::watch(utility::Runnable *runnable)
{
    driver->watch(runnable);
}

TileNode TileSet::Detail::findNode(const TileId &tileId, bool addNew)
    const
{
    // not found in memory -> load from disk
    auto meta(findMetaTile(tileId, addNew));
    if (!meta) { return {}; }

    // now, find node in the metatile
    const MetaNode *node(meta->get(tileId, std::nothrow));
    if (!node && addNew) {
        // no node -> add new
        node = meta->set(tileId, {});
    }

    // return tile node
    return { meta, node };
}

MetaTile::pointer TileSet::Detail::addNewMetaTile(const TileId &tileId) const
{
    auto mid(metaId(tileId));
    LOG(info1) << "Creating metatile " << mid << ".";
    metadataChanged = true;

    // create metatile and store it in tileindex
    return metaTiles->add(std::make_shared<MetaTile>(mid, metaOrder()));
}

MetaTile::pointer TileSet::Detail::findMetaTile(const TileId &tileId
                                                , bool addNew)
    const
{
    TileId mid(metaId(tileId));
    auto meta(metaTiles->find(mid));
    if (meta) { return meta; }

    // does this metatile exist in the index?
    if (!tsi.meta(mid)) {
        if (addNew) { return addNewMetaTile(tileId); }
        return {};
    }

    // some child nodes exist therefore this metatile can exist:
    //     * read-only mode: error if non-existent
    //     * read-write mode (addNew == true): add on failure
    IStream::pointer f;
    if (addNew) {
        f = driver->input(mid, TileFile::meta, NullWhenNotFound);
        if (!f) {
            // metatile doesn't exist -> Create
            return addNewMetaTile(tileId);
        }
    } else {
        f = driver->input(mid, TileFile::meta);
    }
    return metaTiles->add(loadMetaTile(&f->get(), metaOrder(), f->name()));
}

MetaTile::pointer TileSet::Detail::loadMetaTileFor(const TileId &tileId) const
{
    IStream::pointer f(driver->input(metaId(tileId), TileFile::meta));
    return loadMetaTile(&f->get(), metaOrder(), f->name());
}

int TileSet::Detail::getMetaTileVersion(const TileId &tileId) const
{
    IStream::pointer f(driver->input(metaId(tileId), TileFile::meta));
    return MetaTile::loadVersion(f->get(), f->name());
}

const registry::ReferenceFrame& TileSet::referenceFrame() const
{
    return detail().referenceFrame;
}

void TileSet::Detail::save(const OStream::pointer &os, const Mesh &mesh
                           , const Atlas *atlas) const
{
    saveMesh(*os, mesh, atlas);
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
    mesh = loadMesh(os);
}

void TileSet::Detail::load(const IStream::pointer &os, MeshMask &meshMask)
    const
{
    meshMask = loadMeshMask(os);
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

void TileSet::Detail::updateProperties(Lod lod, const MetaNode &metanode
                                       , const MetaNode &oldMetanode)
{
    properties.credits.insert(metanode.credits().begin()
                              , metanode.credits().end());
    add(texelSizeAggregator[lod], metanode, oldMetanode);
    propertiesChanged = true;
}

void TileSet::Detail::updateProperties(const NodeInfo &nodeInfo)
{
    auto fsdsExtents(sdsExtents.find(nodeInfo.srs()));
    if (fsdsExtents == sdsExtents.end()) {
        sdsExtents.insert
            (SdsExtents::value_type
             (nodeInfo.srs(), nodeInfo.extents()));
    } else {
        fsdsExtents->second
            = math::unite(fsdsExtents->second, nodeInfo.extents());
    }
    propertiesChanged = true;
}

void TileSet::Detail::updateProperties(const Mesh &mesh)
{
    for (const auto &sm : mesh) {
        if (sm.textureLayer) {
            properties.boundLayers.insert(*sm.textureLayer);
        }
    }
}

namespace {

std::uint8_t flagsFromNode(const MetaNode &node)
{
    std::uint8_t m(0);
    if (node.geometry()) {
        // alien flag allowed only in when we have a mesh
        m |= TileIndex::Flag::mesh;
        if (node.alien()) { m |= TileIndex::Flag::alien; }
    }
    if (node.navtile()) { m |= TileIndex::Flag::navtile; }
    if (node.internalTextureCount()) { m |= TileIndex::Flag::atlas; }
    return m;
}

} // namespace

void TileSet::Detail::updateNode(TileId tileId, const MetaNode &metanode
                                 , TileIndex::Flag::value_type extraFlags)
{
    // get node (create if necessary)
    auto node(findNode(tileId, true));

    // collect global information (i.e. credits) and texel size
    updateProperties(tileId.lod, metanode, *node.metanode);

    // update node value
    node.update(tileId, metanode);

    // prepare tileindex flags and mask
    auto mask(TileIndex::Flag::content | TileIndex::Flag::nonmeta
              | TileIndex::Flag::alien);
    auto flags(flagsFromNode(*node.metanode));
    flags |= extraFlags;

    // set tile index
    tileIndex.setMask(tileId, mask, flags);

    // go up the tree
    while (tileId.lod) {
        auto parentId(parent(tileId));
        auto parentNode(findNode(parentId, true));

        auto mn(*parentNode.metanode);
        parentNode.set(parentId, mn.setChildFromId(tileId)
                       .mergeExtents(*node.metanode));

        // next round
        tileId = parentId;
        node = parentNode;
    }
}

bool check(const SdsExtents &l, const SdsExtents &r)
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
                << ": there cannot be more textures (" << atlas->size()
                << ") than sub-meshes (" << mesh->size() << ").";
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

        if (!nodeInfo.node().externalTexture) {
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

        // use/compute geom extents
        if (vts::empty(tile.geomExtents)) {
            // no geom extents, need to compute from mesh converted to SDS
            metanode.geomExtents = geomExtents
                (CsConvertor(referenceFrame.model.physicalSrs, nodeInfo.srs())
                 , *mesh);
        } else {
            metanode.geomExtents = tile.geomExtents;
        }

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

        // set alien flag
        metanode.alien(tile.alien);
    }

    // navtile
    if (navtile) {
        metanode.navtile(true);
        metanode.heightRange = navtile->heightRange();
    }

    // store node
    updateNode(tileId, metanode, vts::extraFlags(mesh));

    // save data
    if (mesh) {
        save(driver->output(tileId, TileFile::mesh), *mesh, atlas);
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
                              , const NodeInfo *ni)
{
    const auto nodeInfo(ni ? *ni : NodeInfo(referenceFrame, tileId));

    // store node
    updateNode(tileId, tile.metanode, tile.extraFlags);

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
    updateProperties(nodeInfo);
}

void TileSet::Detail::setNavTile(const TileId &tileId, const NavTile &navtile)
{

    auto node(findNode(tileId));
    if (!node || !node.metanode->geometry()) {
        LOGTHROW(err2, storage::NoSuchTile)
            << "Cannot set navtile to geometry-less tile " << tileId << ".";
    }

    LOG(info1) << "Setting navtile: " << tileId;

    save(driver->output(tileId, TileFile::navtile), navtile);
    auto metanode(*node.metanode);

    // mark as having navtile and update height range
    metanode.navtile(true);
    metanode.heightRange = navtile.heightRange();

    // update the tree
    node.update(tileId, metanode);

    // mark navtile in tile index
    tileIndex.setMask(tileId, TileIndex::Flag::navtile);
}

void TileSet::Detail::setSurrogateValue(const TileId &tileId, float value)
{
    auto node(findNode(tileId));
    if (!node || !node.metanode->geometry()) {
        LOGTHROW(err2, storage::NoSuchTile)
            << "Cannot set surrogate to geometry-less tile " << tileId << ".";
    }

    LOG(info1) << "Setting surrogate (" << tileId << "): " << value;
    auto metanode(*node.metanode);
    metanode.geomExtents.surrogate = value;

    // update the tree
    node.update(tileId, metanode);
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

Mesh TileSet::Detail::getMesh(const TileId &tileId
                              , TileIndex::Flag::value_type flags)
    const
{
    if (!TileIndex::Flag::isReal(flags)) {
        LOGTHROW(err2, storage::NoSuchTile)
            << "There is no tile at " << tileId << ".";
    }

    if (!(flags & TileIndex::Flag::mesh)) {
        LOGTHROW(err2, storage::NoSuchTile)
            << "Tile " << tileId << " has no mesh.";
    }

    Mesh mesh;
    load(driver->input(tileId, TileFile::mesh), mesh);
    return mesh;
}

MeshMask TileSet::Detail::getMeshMask(const TileId &tileId
                                      , const MetaNode *node
                                      , bool generate)
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

    MeshMask meshMask;
    if (!generate) {
        // OK, we can return stored mesh
        load(driver->input(tileId, TileFile::mesh), meshMask);
    } else {
        // generate fresh mask from mesh
        NodeInfo ni(referenceFrame, tileId);

        Mesh mesh;
        load(driver->input(tileId, TileFile::mesh), mesh);

        // convert to SDS
        CsConvertor conv(referenceFrame.model.physicalSrs, ni.srs());
        for (auto &sm : mesh.submeshes) {
            for (auto &v : sm.vertices) { v = conv(v); }
        }

        generateMeshMask(meshMask, mesh, ni.extents());
    }
    return meshMask;
}

MeshMask TileSet::Detail::getMeshMask(const TileId &tileId, bool generate)
    const
{
    return getMeshMask(tileId, findMetaNode(tileId), generate);
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

void TileSet::Detail::getAtlas(const TileId &tileId, Atlas &atlas
                               , TileIndex::Flag::value_type flags) const
{
    if (!TileIndex::Flag::isReal(flags)) {
        LOGTHROW(err2, storage::NoSuchTile)
            << "There is no tile at " << tileId << ".";
    }

    if (!(flags & TileIndex::Flag::atlas)) {
        LOGTHROW(err2, storage::NoSuchTile)
            << "Tile " << tileId << " has no atlas.";
    }

    load(driver->input(tileId, TileFile::atlas), atlas);
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

    // rw -> no tileindex generated -> find metatile's node
    if (auto node = findNode(tileId)) {
        return node.metanode->real();
    }
    return false;
}

void TileSet::Detail::saveMetadata()
{

    driver->wannaWrite("save metadata");
    metaTiles->save();
    saveTileIndex();
}

void update(TileSet::Properties &properties, const TileIndex &tileIndex)
{
    auto ranges(tileIndex.ranges(TileIndex::Flag::mesh));
    properties.lodRange = ranges.first;
    properties.tileRange = ranges.second;
}

void TileSet::Detail::flush()
{
    LOG(info2) << "Flushing <" << properties.id << ">.";
    driver->wannaWrite("flush");

    if (metadataChanged) {
        saveMetadata();
        metadataChanged = false;
        // force properties change
        propertiesChanged = true;

        if (!properties.position.valid() && !sdsExtents.empty()) {
            // guess position from spatial division extents
            // TODO: convert to navigation SRS and make a union
            const auto &item(*sdsExtents.begin());
            auto &position(properties.position);

            position.type = registry::Position::Type::objective;
            position.orientation = { .0, -90., .0 };
            position.heightMode = registry::Position::HeightMode::fixed;

            math::Point2 p(center(item.second));
            position.position = { p(0), p(1), 1000 };
            position.verticalExtent = 5000;
            position.verticalFov = registry::Position::naturalFov();

            CsConvertor conv(item.first, referenceFrame.model.navigationSrs);
            position.position = conv(position.position);
        }
    }

    // update and save config
    if (propertiesChanged) {
        if (const auto avg = average(texelSizeAggregator[lodRange.max])) {
            properties.nominalTexelSize = avg;
        }

        update(properties, tileIndex);
        saveConfig();
        propertiesChanged = false;
    }

    // flush driver; makes read-only
    driver->flush();

    // new cache - now readonly
    metaTiles = MetaCache::create(driver);
}

void TileSet::Detail::emptyCache() const
{
    LOG(info1) << "Emptying cache in <" << properties.id << ">.";

    if (metadataChanged || propertiesChanged) {
        LOGTHROW(err2, storage::PendingTransaction)
            << "Cannot empty cached data due to unflushed changes. "
            << "Use flush instead.";
    }

    // drop all metatiles from memory cache
    metaTiles->clear();
}

const Driver& TileSet::driver() const
{
    return *detail().driver;
}

fs::path TileSet::root() const
{
    return detail().driver->root();
}

MapConfig TileSet::mapConfig(bool includeExtra) const
{
    return detail().mapConfig(includeExtra);
}

MeshTilesConfig TileSet::meshTilesConfig(bool includeExtra) const
{
    return detail().meshTilesConfig(includeExtra);
}

ExtraTileSetProperties TileSet::Detail::loadExtraConfig() const
{
    return loadExtraConfig(*driver);
}

ExtraTileSetProperties TileSet::Detail::loadExtraConfig(const Driver &driver)
{
    if (auto is = driver.input(File::extraConfig, NullWhenNotFound)) {
        return tileset::loadExtraConfig(*is);
    }
    return {};
}

registry::RegistryBase TileSet::Detail::loadRegistry() const
{
    return loadRegistry(*driver);
}

registry::RegistryBase TileSet::Detail::loadRegistry(const Driver &driver)
{
    if (auto is = driver.input(File::registry, NullWhenNotFound)) {
        registry::RegistryBase rb;
        registry::load(rb, *is);
        return rb;
    }
    return {};
}

const TileIndex& TileSet::tileIndex() const
{
    return detail().tileIndex;
}

TileIndex TileSet::metaIndex() const
{
    return detail().tsi.deriveMetaIndex();
}

MapConfig TileSet::mapConfig(const boost::filesystem::path &root
                             , bool includeExtra)
{
    return Detail::mapConfig(*Driver::configReader(root), includeExtra);
}

MapConfig TileSet::mapConfig(const Driver &driver, bool includeExtra)
{
    return Detail::mapConfig(driver, includeExtra);
}

MapConfig TileSet::Detail::mapConfig(const Driver &driver, bool includeExtra)
{
    return vts::mapConfig(tileset::loadConfig(driver)
                          , loadRegistry(driver)
                          , (includeExtra ? loadExtraConfig(driver)
                             : ExtraTileSetProperties()));
}

MapConfig TileSet::Detail::mapConfig(bool includeExtra) const
{
    return vts::mapConfig(properties
                          , loadRegistry()
                          , (includeExtra ? loadExtraConfig()
                             : ExtraTileSetProperties()));
}

MapConfig mapConfig(const FullTileSetProperties &properties
                    , const registry::RegistryBase &localRegistry
                    , const ExtraTileSetProperties &extra
                    , const boost::optional<boost::filesystem::path> &root)
{
    auto referenceFrame(registry::system.referenceFrames
                        (properties.referenceFrame));

    // initialize mapconfif with local registry (NB: mapconfig is full blown
    // registry with some extra stuff)
    MapConfig mapConfig(localRegistry);

    // prefill with extra entitities
    mapConfig.credits.update(extra.credits);
    mapConfig.boundLayers.update(extra.boundLayers);
    mapConfig.freeLayers = extra.freeLayers;
    mapConfig.bodies.update(extra.bodies);

    // build
    mapConfig.referenceFrame = referenceFrame;
    mapConfig.bodies.update(registry::listBodies(referenceFrame));
    mapConfig.srs = registry::listSrs(referenceFrame);
    mapConfig.credits.update(registry::creditsAsDict(properties.credits));
    mapConfig.boundLayers.update
        (registry::boundLayersAsDict(properties.boundLayers));

    mapConfig.surfaces.emplace_back();
    auto &surface(mapConfig.surfaces.back());
    surface.id = properties.id;
    surface.revision = properties.revision;
    surface.has2dInterface = true;

    if (root) {
        surface.root = *root;
    } else {
        driver::MapConfigOverride mco(properties.driverOptions);
        surface.root = mco.root;
    }

    if (!properties.lodRange.empty()) {
        surface.lodRange = properties.lodRange;
    }

    if (valid(properties.tileRange)) {
        surface.tileRange = properties.tileRange;
    }

    // TODO: add
    surface.textureLayer = extra.textureLayer;

    mapConfig.position
        = extra.position ? *extra.position : properties.position;

    mapConfig.rois = extra.rois;

    mapConfig.namedViews = extra.namedViews;

    if (extra.view) {
        // use settings from extra config
        mapConfig.view = extra.view;
    } else {
        // just one surface in the view
        mapConfig.view.addSurface(surface.id);
        mapConfig.view.addBodies
            (registry::listParentBodies(referenceFrame));
    }

    mapConfig.browserOptions = extra.browserOptions;

    return mapConfig;
}

MeshTilesConfig TileSet::meshTilesConfig(const boost::filesystem::path &root
                                         , bool includeExtra)
{
    return Detail::meshTilesConfig(*Driver::open(root), includeExtra);
}

MeshTilesConfig TileSet::meshTilesConfig(const Driver &driver
                                         , bool includeExtra)
{
    return Detail::meshTilesConfig(driver, includeExtra);
}

MeshTilesConfig TileSet::Detail::meshTilesConfig(const Driver &driver
                                                 , bool includeExtra)
{
    return vts::meshTilesConfig(tileset::loadConfig(driver)
                                , (includeExtra ? loadExtraConfig(driver)
                                   : ExtraTileSetProperties()));
}

MeshTilesConfig TileSet::Detail::meshTilesConfig(bool includeExtra) const
{
    return vts::meshTilesConfig(properties, (includeExtra ? loadExtraConfig()
                                             : ExtraTileSetProperties()));
}

MeshTilesConfig
meshTilesConfig(const FullTileSetProperties &properties
                , const ExtraTileSetProperties &extra
                , const boost::optional<boost::filesystem::path> &root)
{
    MeshTilesConfig config;

    // reference frame
    config.referenceFrame = properties.referenceFrame;

    // prefill with extra entitities
    config.credits = extra.credits;

    // build
    config.credits.update(registry::creditsAsDict(properties.credits));

    config.surface.id = properties.id;
    config.surface.revision = properties.revision;

    if (root) {
        config.surface.root = *root;
    } else {
        driver::MapConfigOverride mco(properties.driverOptions);
        config.surface.root = mco.root;
    }

    if (!properties.lodRange.empty()) {
        config.surface.lodRange = properties.lodRange;
    }

    if (valid(properties.tileRange)) {
        config.surface.tileRange = properties.tileRange;
    }

    return config;
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
    return Driver::check(root);
}

bool TileSet::check(const boost::filesystem::path &root
                    , const std::string &mime)
{
    return Driver::check(root, mime);
}

std::shared_ptr<Driver>
TileSet::openDriver(const boost::filesystem::path &root
                    , const OpenOptions &openOptions)
{
    return Driver::open(root, openOptions);
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

TileIndex::Flag::value_type TileSet::Detail::extraFlags(const TileId &tileId)
    const
{
    return (tileIndex.get(tileId) & TileIndex::Flag::nonmeta);
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

        if (!metanode->real()) { return; }

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

        if (mask & TileIndex::Flag::navtile) {
            // copy navtile if allowed
            copyFile(sd.input(tid, storage::TileFile::navtile)
                     , dd.output(tid, storage::TileFile::navtile));
        }

        // TODO: do not copy influenced flag if we have not copied any tile that
        // influences this one

        dst.updateNode(tid, *metanode
                       , mask & TileIndex::Flag::nonmeta);
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
                LOGTHROW(err1, vtslibs::storage::IncompatibleTileSet)
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
    // specified by user
    for (; !tsList.empty(); tsList.pop_back()) {
        dst.paste(tsList.back(), createOptions.lodRange());
    }

    dst.flush();

    return dst;
}

std::string TileSet::typeInfo() const
{
    return driver().info();
}

TileId TileSet::metaId(const TileId &tileId) const
{
    return detail().metaId(tileId);
}

void TileSet::relocate(const boost::filesystem::path &root
                       , const RelocateOptions &options
                       , const std::string &prefix)
{
    Driver::relocate(root, options, prefix);
}

void TileSet::reencode(const boost::filesystem::path &root
                       , const ReencodeOptions &options
                       , const std::string &prefix)
{
    Driver::reencode(root, options, prefix);
}

NodeInfo TileSet::nodeInfo(const TileId &tileId) const
{
    return NodeInfo(detail().referenceFrame, tileId);
}

double TileSet::texelSize() const
{
    return detail().texelSize();
}

double TileSet::Detail::texelSize() const
{
    if (properties.nominalTexelSize) { return *properties.nominalTexelSize; }

    const auto metaIndex(tsi.deriveMetaIndex(lodRange.max));
    const auto mbo(referenceFrame.metaBinaryOrder);

    // aggregate texel sizes
    TexelSizeAggregator aa;

    traverse(metaIndex, lodRange.max, [&](TileId tid, QTree::value_type)
    {
        // expand shrinked metatile identifiers
        tid.x <<= mbo;
        tid.y <<= mbo;
        loadMetaTileFor(tid)->for_each([&](const TileId&, const MetaNode &node)
                                       -> void
        {
            add(aa, node);
        });
    });

    return average(aa);
}

NodeInfo TileSet::rootNode() const {
    return NodeInfo(detail().referenceFrame);
}

void TileSet::Detail::markInfluencedTile(const TileId &tileId)
{
    const auto flags(tileIndex.get(tileId));
    if (TileIndex::Flag::isReal(flags)) {
        LOGTHROW(err2, storage::Error)
            << "Tile <" << tileId
            << " cannot be marked as an influenced tile since it "
            "contains data";
    }

    // ensure there is (and empty) metanode
    findNode(tileId, true);
    // mark
    tileIndex.set(tileId, flags | TileIndex::Flag::influenced);
}

void TileSet::markInfluencedTile(const TileId &tileId)
{
    detail().markInfluencedTile(tileId);
}

unsigned int TileSet::ensureRevision(unsigned int revision)
{
    auto &d(detail());
    if (revision > d.properties.revision) {
        d.properties.revision = revision;
        d.propertiesChanged = true;
    }
    return d.properties.revision;
}

unsigned int TileSet::ensureRevision(const boost::filesystem::path &root)
{
    if (const auto oldRevision = Driver::oldRevision(root)) {
        return ensureRevision(*oldRevision + 1);
    }
    return detail().properties.revision;
}

} } // namespace vtslibs::vts
