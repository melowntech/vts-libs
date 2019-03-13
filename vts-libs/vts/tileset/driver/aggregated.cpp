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

/** Aggregated driver: on the fly surface aggregator
 *
 * TODO: compose registry from all sub tilesets
 */

#include <stdexcept>
#include <limits>
#include <type_traits>
#include <algorithm>
#include <fstream>
#include <mutex>

#include <boost/filesystem.hpp>
#include <boost/format.hpp>
#include <boost/utility/in_place_factory.hpp>
#include <boost/uuid/string_generator.hpp>
#include <boost/uuid/random_generator.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <boost/range/adaptor/reversed.hpp>

#include "dbglog/dbglog.hpp"

#include "utility/streams.hpp"
#include "utility/path.hpp"
#include "utility/binaryio.hpp"
#include "utility/openmp.hpp"
#include "utility/progress.hpp"

#include "../../../storage/error.hpp"
#include "../../../storage/fstreams.hpp"
#include "../../../storage/sstreams.hpp"
#include "../../../storage/io.hpp"
#include "../../io.hpp"
#include "../../tileflags.hpp"
#include "../config.hpp"
#include "../detail.hpp"
#include "aggregated.hpp"
#include "runcallback.hpp"

namespace vtslibs { namespace vts { namespace driver {

namespace fs = boost::filesystem;

namespace vs = vtslibs::storage;

namespace {

const std::string ConfigName("tileset.conf");
const std::string ExtraConfigName("extra.conf");
const std::string TileIndexName("tileset.index");
const std::string RegistryName("tileset.registry");

inline const std::string filePath(File type)
{
    switch (type) {
    case File::config: return ConfigName;
    case File::extraConfig: return ExtraConfigName;
    case File::tileIndex: return TileIndexName;
    case File::registry: return RegistryName;
    default: break;
    }
    throw "unknown file type";
}

void unite(registry::IdSet &out, const registry::IdSet &in)
{
    out.insert(in.begin(), in.end());
}


typedef TileIndex::Flag TiFlag;
typedef TiFlag::value_type value_type;

void addTileIndex(unsigned int metaBinaryOrder
                  , TileIndex &ti, AggregatedDriver::DriverEntry &de
                  , MetaNode::SourceReference setId, bool alien)
{
    auto combiner([&](value_type o, value_type n) -> value_type
    {
        if (o & TiFlag::mesh) {
            // already occupied by existing tile
            return o;
        }

        if (alien != TiFlag::isAlien(n)) {
            // different alien flag
            return o;
        }

        if (n & TiFlag::mesh) {
            // combine flags with set id
            return (n & TiFlag::nonAlien) | (setId << 16);
        }

        return o;
    });

    // process tileindex
    tileset::Index work(metaBinaryOrder);
    tileset::loadTileSetIndex(work, *de.driver);
    ti.combine(work.tileIndex, combiner);

    // derive metatile index from tileset's tile index
    if (!alien) {
        // mark metatile presence only for content tiles
        de.metaIndex = work.deriveMetaIndex(true);
    }
}

AggregatedDriver::DriverEntry::list
openDrivers(Storage &storage, const OpenOptions &openOptions
            , const AggregatedOptions &options)
{
    // get list of tilesets (in proper order)
    const auto tilesets(storage.tilesets(options.tilesets));

    const auto tsMap(deserializeTsMap(options.tsMap));
    AggregatedDriver::DriverEntry::list drivers;

    drivers.reserve(tsMap.size());

    for (const auto &references : tsMap) {
        // construct glue id
        // TODO: check for proper index
        Glue::Id glueId;
        for (auto reference : references) {
            if (reference >= tilesets.size()) {
                LOGTHROW(err1, vs::Corrupted)
                    << "Invalid tileset reference <" << reference << ">.";
            }
            glueId.push_back(tilesets[reference]);
        }

        if (glueId.size() == 1) {
            // tileset
            LOG(info1) << "opening <" << glueId.front() << ">";
            drivers.emplace_back(Driver::open(storage.path(glueId.front())
                                              , openOptions)
                                 , references);
        } else {
            // glue
            LOG(info1) << "opening <" << utility::join(glueId, ",") << ">";
            drivers.emplace_back(Driver::open(storage.path(glueId)
                                              , openOptions)
                                 , references);
        }
    }

    return drivers;
}

bool isAsync(const AggregatedDriver::DriverEntry::list &drivers) {
    for (const auto &de : drivers) {
        if (de.driver->ccapabilities().async) { return true; }
    }
    return false;
}

struct PreparedDrivers {
    AggregatedDriver::DriverEntry::list drivers;
    std::vector<fs::path> paths;
};

/** Builds the same structure as openDrivers but no driver is physically
 * created.
 */
PreparedDrivers
prepareDrivers(Storage &storage, const AggregatedOptions &options)
{
    // get list of tilesets (in proper order)
    const auto tilesets(storage.tilesets(options.tilesets));

    const auto tsMap(deserializeTsMap(options.tsMap));

    PreparedDrivers preparedDrivers;
    auto &drivers(preparedDrivers.drivers);
    auto &paths(preparedDrivers.paths);

    drivers.reserve(tsMap.size());

    for (const auto &references : tsMap) {
        // construct glue id
        // TODO: check for proper index
        Glue::Id glueId;
        for (auto reference : references) {
            if (reference >= tilesets.size()) {
                LOGTHROW(err1, vs::Corrupted)
                    << "Invalid tileset reference <" << reference << ">.";
            }
            glueId.push_back(tilesets[reference]);
        }

        if (glueId.size() == 1) {
            // tileset
            LOG(info1) << "preparing <" << glueId.front() << ">";
            drivers.emplace_back(references);
            paths.push_back(storage.path(glueId.front()));
        } else {
            // glue
            LOG(info1) << "preparing <" << utility::join(glueId, ",") << ">";
            drivers.emplace_back(references);
            paths.push_back(storage.path(glueId));
        }
    }

    return preparedDrivers;
}

IStream::pointer
buildMeta(const AggregatedDriver::DriverEntry::list &drivers
          , const fs::path &root
          , const registry::ReferenceFrame &referenceFrame
          , std::time_t lastModified, const TileId &tileId
          , const TileIndex &tileIndex, bool keepSurfaceReferences
          , bool noSuchFile = true)
{
    const auto mbo(referenceFrame.metaBinaryOrder);
    // parent tile at meta-binary-order levels above us
    const auto parentId(parent(tileId, mbo));
    // shrinked tile id to be used in meta-index
    const TileId shrinkedId(tileId.lod, parentId.x, parentId.y);

    // load metatile, doesn't fail
    auto loadMeta([&](const TileId &tileId, const Driver::pointer &driver)
                  -> MetaTile
    {
        auto ms(driver->input(tileId, TileFile::meta, NullWhenNotFound));
        if (!ms) {
            // not found -> empty metanode (at right place)

            // TODO: check for node validity; all metatile nodes invalid ->
            // fine, empty metatile is ok; otherwise it should be an error
            return MetaTile(tileId, mbo);
        }
        return loadMetaTile(*ms, mbo, ms->name());
    });

    // output metatile
    MetaTile ometa(tileId, mbo);

    if (const auto *tree = tileIndex.tree(tileId.lod)) {
        tree->forEach
            (parentId.lod, parentId.x, parentId.y
             , [&](unsigned int x, unsigned int y, QTree::value_type value)
        {
            ometa.expectReference
                (TileId(tileId.lod, tileId.x + x, tileId.y + y)
                 , sourceReferenceFromFlags(value));
        }, QTree::Filter::white);
    }

    // start from zero so first round gets 1
    int idx(0);
    for (const auto &de : drivers) {
        // next index
        ++idx;

        // check for metatile existence
        if (!de.metaIndex.get(shrinkedId)) { continue; }

        // load metatile from this tileset and update output metatile
        ometa.update(idx, loadMeta(tileId, de.driver));
    }

    if (ometa.empty()) {
        if (noSuchFile) {
            LOGTHROW(err1, vs::NoSuchFile)
                << "There is no metatile for " << tileId << ".";
        }
        LOG(err1) << "There is no metatile for " << tileId << ".";
        return {};
    }

    // generate child flags based on tile index
    // TODO: make use value computed by ometa.update above
    ometa.for_each([&](const TileId &nodeId, MetaNode &node)
    {
        // create nodeinfo for this node
        // TODO: optimize, still takes too long
        NodeInfo ni(referenceFrame, nodeId);
        if (!ni.valid()) { return; }

        if (!keepSurfaceReferences) {
            // do not keep surface references -> reset
            node.sourceReference = 0;
        }

        // DEBUG: remember accumulate child flags
        auto cf(node.childFlags());

        for (const auto &child : vts::children(nodeId)) {
            bool valid(tileIndex.validSubtree(child)
                       && ni.child(child).valid());
            node.setChildFromId(child, valid);
        }

        // DEBUG: compare accumulated flags with computed flags
        if (cf != node.childFlags()) {
            LOG(warn2)
                << "Aggregated tileset " << root
                << ": " << nodeId << " has mismatching child flags "
                << "accumulated: " << std::bitset<8>(cf) << ", computed: "
                << std::bitset<8>(node.childFlags()) << ".";
        }
    });

    // create stream and serialize metatile

    // create in-memory stream
    auto fname(root / str(boost::format("%s.%s") % tileId % TileFile::meta));
    auto s(std::make_shared<StringIStream>
           (TileFile::meta, fname.string(), lastModified));

    // and serialize metatile
    ometa.save(s->sink());
    s->updateSize();

    // done
    return s;
}

inline std::unique_ptr<Cache>
createCache(const fs::path &root
            , const boost::optional<PlainOptions> &metaOptions
            , bool readOnly = true)
{
    if (!metaOptions) { return {}; }
    return std::unique_ptr<Cache>(new Cache(root, *metaOptions, readOnly));
}

} // namespace

fs::path AggregatedOptions::buildStoragePath(const fs::path &root) const
{
    return fs::absolute(storagePath, root);
}

AggregatedDriverBase::AggregatedDriverBase(const CloneOptions &cloneOptions)
{
    if (!cloneOptions.tilesetId()) {
        LOGTHROW(err2, storage::NoSuchTileSet)
            << "Attempt to create aggregated driver without providing "
            "tilesetId.";
    }
}

/** Create driver for on-disk tileset
 */
AggregatedDriver::AggregatedDriver(const boost::filesystem::path &root
                                   , const AggregatedOptions &options
                                   , const CloneOptions &cloneOptions)
    : AggregatedDriverBase(cloneOptions)
    , Driver(root, cloneOptions.openOptions(), options, cloneOptions.mode())
    , storage_(this->options().buildStoragePath(root)
               , OpenMode::readOnly)
    , referenceFrame_(storage_.referenceFrame())
    , tsi_(referenceFrame_.metaBinaryOrder, drivers_, &options)
    , surfaceReferences_(this->options().surfaceReferences)
{
    // we flatten the content
    capabilities().flattener = true;
    capabilities().async = isAsync(drivers_);

    // build driver information
    auto properties(build(options, cloneOptions, true));

    // save stuff (allow write for a brief moment)
    tileset::saveConfig(this->root() / filePath(File::config), properties);
    tileset::saveTileSetIndex(tsi_, *this);

    readOnly(true);
}

/** Create driver for in-memory tileset
 */
AggregatedDriver::AggregatedDriver(const AggregatedOptions &options
                                   , const CloneOptions &cloneOptions)
    : AggregatedDriverBase(cloneOptions)
    , Driver(cloneOptions.openOptions(), options, cloneOptions.mode())
    , storage_(this->options().storagePath, OpenMode::readOnly)
    , referenceFrame_(storage_.referenceFrame())
    , tsi_(referenceFrame_.metaBinaryOrder, drivers_)
    , surfaceReferences_(this->options().surfaceReferences)
{
    // we flatten the content
    capabilities().flattener = true;

    // build driver information and cache it
    memProperties_ = build(options, cloneOptions);

    capabilities().async = isAsync(drivers_);
}

void AggregatedDriver::Index::loadRest_impl(std::istream &f
                                            , const fs::path &path)
{
    for (auto &de : drivers_) {
        LOG(debug) << "Loading tileset metaindex.";
        de.metaIndex.load(f, path);
    }
}

void AggregatedDriver::Index::saveRest_impl(std::ostream &f) const
{
    for (const auto &de : drivers_) {
        LOG(debug) << "Saving tileset metaindex.";
        de.metaIndex.save(f, TileIndex::SaveParams().bw(true));
    }
}

TileSet::Properties AggregatedDriver::build(AggregatedOptions options
                                            , const CloneOptions &cloneOptions
                                            , bool onDisk)
{
    auto pendingGlues = storage_.pendingGlues(&options.tilesets);
    if (!pendingGlues.empty()) {
        LOG(err2) << "Cannot aggregate tilesets: pending glues.";
        throw PendingGluesError(pendingGlues);
    }

    TileSet::Properties properties;
    properties.id = *cloneOptions.tilesetId();

    // try to get previous revision (and reuse)
    if (auto oldR = oldRevision()) {
        properties.revision = *oldR + 1;
    }

    // get reference frame
    properties.referenceFrame = storage_.getProperties().referenceFrame;
    properties.tileRange = TileRange(math::InvalidExtents{});
    properties.lodRange = LodRange::emptyRange();

    // compose tile index and other properties
    TileIndex &ti(tsi_.tileIndex);

    LOG(info1) << "Building tileset info";
    // Step #1: grab all allowed tilesets and their glues
    const auto &tilesets(options.tilesets);

    // grab tilesets and their glues
    TileSetGlues::list tilesetInfo;

    // make room for all tilesets in the output to ensure pointer are not
    // invalidated later
    tilesetInfo.reserve(tilesets.size());

    // we are processing tileset from bottom up
    {
        typedef std::map<TilesetId, TileSetGlues*> BackMap;
        BackMap backMap;
        for (const auto &tilesetId : storage_.tilesets()) {
            if (!tilesets.count(tilesetId)) { continue; }

            tilesetInfo.emplace_back
                (tilesetId, storage_.glues
                 (tilesetId, [&](const Glue::Id &glueId) -> bool
            {
                for (const auto &id : glueId) {
                    if (!tilesets.count(id)) { return false; }
                }
                return true;
            }));

            // remember tileset in back-map
            backMap.insert(BackMap::value_type
                           (tilesetId, &tilesetInfo.back()));
        }

        // Step #2: distribute (possible) aliens in appropriate secondary
        // tilesets
        for (auto &tsi : tilesetInfo) {
            for (const auto &glue : tsi.glues) {
                // sane ID?
                if (glue.id.size() < 2) { continue; }

                const auto &secondaryId(glue.id[glue.id.size() - 2]);
                auto fbackMap(backMap.find(secondaryId));
                if (fbackMap == backMap.end()) {
                    // should this ever happen?
                    continue;
                }

                auto &secTs(*fbackMap->second);

                LOG(debug) << "Adding <" << utility::join(glue.id, ",")
                           << "> as an alien glue in tileset <"
                           << secTs.tilesetId << ">.";

                secTs.glues.push_back(glue);
            }
        }
    }

    typedef AggregatedDriver::DriverEntry DriverEntry;
    typedef AggregatedDriver::TilesetReferences TilesetReferences;

    typedef std::map<TilesetId, int> TilesetId2Index;
    // sort glues
    auto go(glueOrder(tilesetInfo));
    std::reverse(go.begin(), go.end());

    // fill in tileset ID to tileset index
    TilesetId2Index tilesetId2Index;
    {
        int id(0);
        for (const auto &tilesetId : storage_.tilesets(options.tilesets)) {
            tilesetId2Index[tilesetId] = id++;
        }
    }

    // collect total number of real tilesets (regular + glues, without alien
    // glue duplication)
    std::size_t setCount(0);
    for (const auto &tsg : go) {
        ++setCount;

        for (const auto &glue : tsg.glues) {
            if (glue.id.back() == tsg.tilesetId) { ++setCount; }
        }
    }

    const auto tileset2references([&](const TilesetId &tilesetId)
                                  -> TilesetReferences
    {
        return TilesetReferences(1, tilesetId2Index.at(tilesetId));
    });

    const auto glue2references([&](const Glue::Id &glueId) -> TilesetReferences
    {
        TilesetReferences out;
        for (const auto &tilesetId : glueId) {
            out.push_back(tilesetId2Index.at(tilesetId));
        }
        return out;
    });

    // list of drivers and tileset mapping (make room for all instances)
    drivers_.reserve(setCount);

    struct GlueDriver {
        MetaNode::SourceReference setId;
        DriverEntry *de;

        GlueDriver(MetaNode::SourceReference setId = 0
                   , DriverEntry *de = nullptr)
            : setId(setId), de(de)
        {}
    };

    typedef std::map<Glue::Id, GlueDriver> Glue2Driver;
    Glue2Driver glue2driver;

    TilesetReferencesList tsMap;

    const auto mbo(referenceFrame_.metaBinaryOrder);
    bool first(true);
    for (const auto &tsg : go) {
        // first, remember tileset
        drivers_.emplace_back
            (Driver::open(storage_.path(tsg.tilesetId))
             , tileset2references(tsg.tilesetId));
        auto &de(drivers_.back());
        const auto setId(drivers_.size());
        tsMap.push_back(de.tilesets);
        LOG(info2) << "<" << tsg.tilesetId << "> [" << setId << "]";

        // then process all glues
        for (const auto &glue : tsg.glues) {
            bool alien(glue.id.back() != tsg.tilesetId);
            if (!alien) {
                drivers_.emplace_back
                    (Driver::open(storage_.path(glue))
                     , glue2references(glue.id));

                auto &de(drivers_.back());
                tsMap.push_back(de.tilesets);
                glue2driver[glue.id] = GlueDriver(drivers_.size(), &de);

                LOG(info2)
                    << "    <" << utility::join(glue.id, ",") << "> ("
                    << de.driver << ") [" << drivers_.size() << "]";

                // merge-in glue's tile index
                addTileIndex(mbo, ti, de, drivers_.size(), alien);
            } else {
                auto gd(glue2driver.at(glue.id));
                LOG(info2)
                    << "   *<" << utility::join(glue.id, ",") << "> ("
                    << gd.de->driver << ") [" << gd.setId << "]";
                // merge-in glue's tile index
                addTileIndex(mbo, ti, *gd.de, gd.setId, alien);
            }
        }

        // and merge-in tileset's tile index last
        addTileIndex(mbo, ti, de, setId, false);

        // merge-in tileset's config
        {
            const auto tsp(tileset::loadConfig(*de.driver));

            unite(properties.credits, tsp.credits);
            unite(properties.boundLayers, tsp.boundLayers);

            // copy position from first tileset
            if (first) {
                properties.position = tsp.position;
                first = false;
            }
        }
    }

    // update extents
    {
        auto ranges(ti.ranges(TiFlag::mesh));
        properties.lodRange = ranges.first;
        properties.tileRange = ranges.second;
    }

    // set serialized tileset mapping
    options.tsMap = serializeTsMap(tsMap);

    if (onDisk) { generateMetatiles(options); }

    // write options
    properties.driverOptions = options;
    return properties;
}

/** Open driver for existing tileset
 */
AggregatedDriver::AggregatedDriver(const boost::filesystem::path &root
                                   , const OpenOptions &openOptions
                                   , const AggregatedOptions &options)
    : Driver(root, openOptions, options)
    , storage_(this->options().buildStoragePath(root)
               , OpenMode::readOnly)
    , referenceFrame_(storage_.referenceFrame())
    , drivers_(openDrivers(storage_, openOptions, options))
    , tsi_(referenceFrame_.metaBinaryOrder, drivers_, &options)
    , surfaceReferences_(this->options().surfaceReferences)
    , cache_(createCache(root, options.metaOptions))
{
    // we flatten the content
    capabilities().flattener = true;
    capabilities().async = isAsync(drivers_);

    tileset::loadTileSetIndex(tsi_, *this);
}

/** Clone existing tileset
 */
AggregatedDriver::AggregatedDriver(PrivateTag
                                   , const boost::filesystem::path &root
                                   , AggregatedOptions options
                                   , const CloneOptions &cloneOptions
                                   , const AggregatedDriver &src)
    : Driver(root, cloneOptions.openOptions(), options, cloneOptions.mode())
    , storage_(this->options().buildStoragePath(root)
               , OpenMode::readOnly)
    , referenceFrame_(storage_.referenceFrame())
    , tsi_(referenceFrame_.metaBinaryOrder, drivers_, &options)
    , surfaceReferences_(this->options().surfaceReferences)
{
    // we flatten the content
    capabilities().flattener = true;

    // clone tile index
    copyFile(src.input(File::tileIndex), output(File::tileIndex));

    // open drivers
    drivers_ = openDrivers(storage_, cloneOptions.openOptions(), options);
    capabilities().async = isAsync(drivers_);

    // relaod tile index
    tileset::loadTileSetIndex(tsi_, *this);

    // copy metatiles if needed
    if (src.cache_) {
        copyMetatiles(options, src.cache_.get());
    }

    // update and save properties
    {
        auto properties(tileset::loadConfig(src));
        if (cloneOptions.tilesetId()) {
            properties.id = *cloneOptions.tilesetId();
        }
        properties.driverOptions = options;
        tileset::saveConfig(this->root() / filePath(File::config)
                            , properties);
    }

    // clone registry if exists
    if (auto registry = src.input(File::registry, NullWhenNotFound)) {
        copyFile(registry, output(File::registry));
    }

    // make me read-only
    readOnly(true);
}

Driver::pointer
AggregatedDriver::clone_impl(const boost::filesystem::path &root
                             , const CloneOptions &cloneOptions)
    const
{
    return std::make_shared<AggregatedDriver>
        (PrivateTag(), root, options(), cloneOptions, *this);
}

AggregatedDriver::~AggregatedDriver() {}

OStream::pointer AggregatedDriver::output_impl(File type)
{
    if (readOnly()) {
        LOGTHROW(err2, storage::ReadOnlyError)
            << "This driver supports read access only.";
    }

    const auto path(root() / filePath(type));
    LOG(info1) << "Saving to " << path << ".";
    return fileOStream(type, path);
}

IStream::pointer AggregatedDriver::input_mem(File type) const
{
    // this is an in-memory driver

    // no extra config here
    if (type == File::extraConfig) {
        LOGTHROW(err1, storage::NoSuchFile)
            << "No extra config for in-memory driver.";
    }

    auto s(std::make_shared<StringIStream>(type, filePath(type), 0));

    switch (type) {
    case File::config:
        tileset::saveConfig(s->sink(), *memProperties_);
        break;

    case File::tileIndex:
        tileset::saveTileSetIndex(tsi_, s->sink());
        break;

    default: break;
    }

    s->updateSize();

    // done
    return s;
}

IStream::pointer AggregatedDriver::input_impl(File type, bool noSuchFile)
    const
{
    // in-memory -> alternative branch
    if (memProperties_) { return input_mem(type); }

    const auto path(root() / filePath(type));
    LOG(info1) << "Loading from " << path << ".";
    return ((noSuchFile)
            ? fileIStream(type, path)
            : fileIStream(type, path, NullWhenNotFound));
}

OStream::pointer AggregatedDriver::output_impl(const TileId&, TileFile)
{
    LOGTHROW(err2, storage::ReadOnlyError)
        << "This driver supports read access only.";
    return {};
}

IStream::pointer AggregatedDriver::input_impl(const TileId &tileId
                                              , TileFile type
                                              , bool noSuchFile) const
{
    if (type == TileFile::meta) {
        if (!tsi_.meta(tileId)) {
            if (noSuchFile) {
                LOGTHROW(err1, vs::NoSuchFile)
                    << "There is no " << type << " for " << tileId << ".";
            }
            LOG(err1)
                << "There is no " << type << " for " << tileId << ".";
            return {};
        }

        if (cache_ && tsi_.staticMeta(tileId)) {
            if (noSuchFile) {
                return cache_->input(tileId, type);
            }
            return cache_->input(tileId, type, NullWhenNotFound);
        }

        return driver::buildMeta
            (drivers_, root(), referenceFrame_
             , configStat().lastModified, tileId
             , tsi_.tileIndex, surfaceReferences_, noSuchFile);
    }

    const auto flags(tsi_.checkAndGetFlags(tileId, type));
    if (!flags) {
        if (noSuchFile) {
            LOGTHROW(err1, vs::NoSuchFile)
                << "There is no " << type << " for " << tileId << ".";
        }
        LOG(err1)
            << "There is no " << type << " for " << tileId << ".";
        return {};
    }

    // get dataset id
    const auto sourceReference(sourceReferenceFromFlags(flags));

    // NB: source reference is 1-based
    if (sourceReference && (sourceReference <= drivers_.size())) {
        return drivers_[sourceReference - 1].driver->input(tileId, type);
    }

    if (noSuchFile) {
        LOGTHROW(err1, vs::NoSuchFile)
            << "There is no " << type << " for " << tileId << ".";
    }

    LOG(err1) << "There is no " << type << " for " << tileId << ".";
    return {};
}

void reportNotFound(const TileId &tileId, TileFile type
                    , const InputCallback &cb
                    , const IStream::pointer *notFound)
{
    return runCallback([&]() -> IStream::pointer
    {
        if (!notFound) {
            LOGTHROW(err1, vs::NoSuchFile)
                << "There is no " << type << " for " << tileId << ".";
        }
        return *notFound;
    }, cb);
}

void AggregatedDriver::input_impl(const TileId &tileId, TileFile type
                                  , const InputCallback &cb
                                  , const IStream::pointer *notFound) const
{
    if (type == TileFile::meta) {
        if (!tsi_.meta(tileId)) {
            return reportNotFound(tileId, type, cb, notFound);
        }

        if (cache_ && tsi_.staticMeta(tileId)) {
            return runCallback([&]() -> IStream::pointer
            {
                if (notFound) {
                    if (const auto &is
                        = cache_->input(tileId, type, NullWhenNotFound))
                    {
                        return is;
                    }
                    return *notFound;
                }
                return cache_->input(tileId, type);
            }, cb);
        }

        return buildMeta(tileId, configStat().lastModified, cb, notFound);
    }

    const auto flags(tsi_.checkAndGetFlags(tileId, type));
    if (!flags) {
        return reportNotFound(tileId, type, cb, notFound);
    }

    // get dataset id
    const auto sourceReference(sourceReferenceFromFlags(flags));

    // NB: source reference is 1-based
    if (sourceReference && (sourceReference <= drivers_.size())) {
        // TODO: we can call non-async version if appropriate
        return drivers_[sourceReference - 1]
            .driver->input(tileId, type, cb, notFound);
    }

    return reportNotFound(tileId, type, cb, notFound);
}

FileStat AggregatedDriver::stat_impl(File type) const
{
    const auto name(filePath(type));
    const auto path(root() / name);
    LOG(info1) << "Statting " << path << ".";
    return FileStat::stat(path);
}

FileStat AggregatedDriver::stat_impl(const TileId &tileId, TileFile type) const
{
    if (type == TileFile::meta) {
        if (!tsi_.meta(tileId)) {
            LOGTHROW(err1, vs::NoSuchFile)
                << "There is no " << type << " for " << tileId << ".";
            return {};
        }

        if (cache_ && tsi_.staticMeta(tileId)) {
            return cache_->input(tileId, type)->stat();
        }

        return driver::buildMeta
            (drivers_, root(), referenceFrame_
             , configStat().lastModified, tileId
             , tsi_.tileIndex, surfaceReferences_)->stat();
    }

    const auto flags(tsi_.checkAndGetFlags(tileId, type));
    if (!flags) {
        LOGTHROW(err1, vs::NoSuchFile)
            << "There is no " << type << " for " << tileId << ".";
    }

    // get dataset id
    const auto sourceReference(sourceReferenceFromFlags(flags));

    if (sourceReference && (sourceReference <= drivers_.size())) {
        return drivers_[sourceReference - 1].driver->stat(tileId, type);
    }

    LOGTHROW(err1, vs::NoSuchFile)
        << "There is no " << type << " for " << tileId << ".";
    throw;
}

storage::Resources AggregatedDriver::resources_impl() const
{
    // accumulate resources
    storage::Resources resources;
    if (cache_) { resources += cache_->resources(); }
    for (const auto &driver : drivers_) {
        resources += driver.driver->resources();
    }
    return resources;
}

void AggregatedDriver::flush_impl() {
    LOGTHROW(err2, storage::ReadOnlyError)
        << "This driver supports read access only.";
}

void AggregatedDriver::drop_impl()
{
    LOGTHROW(err2, storage::ReadOnlyError)
        << "This driver supports read access only.";
}

std::string AggregatedDriver::info_impl() const
{
    auto o(options());
    std::ostringstream os;
    os << "aggregated[optimized] (storage=" << o.storagePath
       << ", tilesets=[" << utility::join(o.tilesets, " ") << "]";

    if (o.surfaceReferences) {
        os << ", surfaceReferences";
    }

    if (!o.staticMetaRange.empty()) {
        os << ", staticMetaRange=" << o.staticMetaRange;
    }

    os << ")";

    return os.str();
}

boost::any AggregatedOptions::relocate(const RelocateOptions &options
                                       , const std::string &prefix) const
{
    auto res(options.apply(storagePath.string()));

    auto ret([&]() -> boost::any
    {
        if (!res.replacement) {
            LOG(info3)
                << prefix << "Nothing to do with storage "
                << storagePath << ".";
            return {};
        }

        // update
        if (options.dryRun) {
            LOG(info3)
                << prefix << "Would relocate path to storage " << storagePath
                << " to path to storage "
                << *res.replacement << ".";
            return {};
        }

        auto out(*this);
        LOG(info3)
            << prefix << "Relocating local path " << storagePath
            << " to path " << *res.replacement << ".";
        out.storagePath = *res.replacement;
        return out;
    }());

    // follow storage
    Storage::relocate(res.follow, options, prefix + "    ");

    return ret;
}

IStream::pointer AggregatedDriver::input_impl(const std::string &name
                                              , bool noSuchFile) const
{
    if (name == VirtualSurface::TilesetMappingPath) {
        return storage::memIStream
            (VirtualSurface::TilesetMappingContentType
             , options().tsMap, configStat().lastModified, name);
    }

    if (noSuchFile) {
        LOGTHROW(err1, storage::NoSuchFile)
            << "This driver doesn't serve file \"" << name << "\".";
    }
    LOG(err1)
        << "This driver doesn't serve file \"" << name << "\".";
    return {};
}

FileStat AggregatedDriver::stat_impl(const std::string &name) const
{
    if (name == VirtualSurface::TilesetMappingPath) {
        auto stat(configStat());
        stat.size = options().tsMap.size();
        stat.contentType = VirtualSurface::TilesetMappingContentType;
        return stat;
    }

    LOGTHROW(err1, storage::NoSuchFile)
        << "This driver doesn't serve file \"" << name << "\".";
    return {};
}

bool AggregatedDriver::reencode(const boost::filesystem::path &root
                                , const AggregatedOptions &driverOptions
                                , const ReencodeOptions &options
                                , const std::string &prefix)
{
    if (options.descend) {
        Storage::reencode(driverOptions.buildStoragePath(root), options
                          , prefix + "    ");
    }
    return !options.dryRun && !options.cleanup;
}

void AggregatedDriver::generateMetatiles(AggregatedOptions &options)
{
    auto &lodRange(options.staticMetaRange);
    if (lodRange.empty()) { return; }

    LOG(info3) << "Pre-generating metatiles.";

    options.metaOptions = PlainOptions(5, referenceFrame_.metaBinaryOrder);

    // create cache (temporarily read-write)
    cache_ = createCache(root(), options.metaOptions, false);

    // derive this tileset's meta tile index
    const auto mi(tsi_.deriveMetaIndex());

    // clip tile range to available data
    if (lodRange.max > mi.maxLod()) {
        lodRange.max = mi.maxLod();
    }

    auto mbo(tsi_.metaBinaryOrder());

    const utility::Progress::ratio_t reportRatio(5, 1000);
    utility::ts::Progress progress("meta agg", mi.count(lodRange)
                                   , reportRatio);
    auto report([&]() { ++progress; });

    auto getMeta([&](const TileId &tid)
    {
        // get metatile stream, ask for nullptr when metatile doesn't exist
        return driver::buildMeta(drivers_, root(), referenceFrame_
                                 , -1, tid
                                 , tsi_.tileIndex, surfaceReferences_, false);
    });

    // process all metatiles in given range
    UTILITY_OMP(parallel)
    UTILITY_OMP(single)
    traverse(mi, lodRange, [=](TileId tid, QTree::value_type)
    {
        UTILITY_OMP(task)
        {
            // expand shrinked metatile identifiers
            tid.x <<= mbo;
            tid.y <<= mbo;

            // get metatile as a stream
            auto is(getMeta(tid));
            if (is) {
                UTILITY_OMP(critical(vts_driver_aggregated_copy))
                {
                    // file open and write must be under lock since tilar write
                    // is not reentrant (yet)
                    auto os(cache_->output(tid, TileFile::meta));
                    copyFile(is, os);
                }
            } else {
                // no such metatile, probably some tileset lied about tile
                // availability: this can happen for global tilesets generated
                // by mapproxy

                // TODO: check for whole metatile validity (i.e. are all nodes
                // valid)
            }

            report();
        }
    });

    // flush and make readonly
    cache_->flush();
    cache_->makeReadOnly();
}

void AggregatedDriver::copyMetatiles(AggregatedOptions &options
                                     , Cache *srcCache)
{
    options.metaOptions = PlainOptions(5, referenceFrame_.metaBinaryOrder);

    // create cache (temporarily read-write)
    cache_ = createCache(root(), options.metaOptions, false);

    // derive this tileset's meta tile index
    const auto mi(tsi_.deriveMetaIndex());

    const auto &lodRange(options.staticMetaRange);
    auto mbo(tsi_.metaBinaryOrder());

    const utility::Progress::ratio_t reportRatio(1, 1000);
    utility::ts::Progress progress("clone", mi.count(lodRange)
                                   , reportRatio);
    auto report([&]() { ++progress; });

    // process all metatiles in given range
    UTILITY_OMP(parallel)
    UTILITY_OMP(single)
    traverse(mi, lodRange, [=](TileId tid, QTree::value_type)
    {
        UTILITY_OMP(task)
        {
            // expand shrinked metatile identifiers
            tid.x <<= mbo;
            tid.y <<= mbo;

            // build metatile as a stream
            auto is(srcCache->input(tid, TileFile::meta, NullWhenNotFound));

            if (is) {
                UTILITY_OMP(critical(vts_driver_aggregated_copy))
                {
                    // file open and write must be under lock since tilar write
                    // is not reentrant (yet)
                    auto os(cache_->output(tid, TileFile::meta));
                    copyFile(is, os);
                }
            } else {
                // no such metatile, probably some tileset lied about tile
                // availability: this can happen for global tilesets generated
                // by mapproxy

                // TODO: check for whole metatile validity (i.e. are all nodes
                // valid)
            }

            report();
        }
    });

    // flush and make readonly
    cache_->flush();
    cache_->makeReadOnly();
}

// Asynchronous interface

AggregatedDriver::AggregatedDriver(PrivateTag
                                   , const boost::filesystem::path &root
                                   , const OpenOptions &openOptions
                                   , const AggregatedOptions &options
                                   , const Dependencies &dependencies)
    : Driver(root, openOptions, options)
    , storage_(*dependencies.storage)
    , referenceFrame_(storage_.referenceFrame())
    , drivers_(dependencies.drivers)
    , tsi_(referenceFrame_.metaBinaryOrder, drivers_, &options)
    , surfaceReferences_(this->options().surfaceReferences)
    , cache_(createCache(root, options.metaOptions))
{
    // we flatten the content
    capabilities().flattener = true;
    capabilities().async = isAsync(drivers_);

    tileset::loadTileSetIndex(tsi_, *this);
}

void AggregatedDriver::open(const boost::filesystem::path &root
                            , const OpenOptions &openOptions
                            , const AggregatedOptions &options
                            , const DriverOpenCallback::pointer &callback)
{
    struct DoCallback
        : std::enable_shared_from_this<DoCallback>
        , StorageOpenCallback
    {
        typedef std::shared_ptr<DoCallback> pointer;

        DoCallback(const boost::filesystem::path &root
                   , const OpenOptions &openOptions
                   , const AggregatedOptions &options
                   , DriverOpenCallback::pointer callback)
            : root(root), openOptions(openOptions)
            , options(options), callback(std::move(callback))
            , expectDrivers()
        {}

        struct DriverOpener : DriverOpenCallback {
            DriverOpener(DoCallback::pointer owner
                         , AggregatedDriver::DriverEntry &de)
                : owner(owner), de(de)
            {}

            virtual void done(Driver::pointer driver) {
                de.driver = std::move(driver);

                owner->newDriver();
            }

            virtual void error(const std::exception_ptr &exc) {
                owner->error(exc);
            }

            virtual void openStorage(const boost::filesystem::path &path
                                     , const StorageOpenCallback::pointer
                                     &callback)
            {
                return owner->callback->openStorage(path, callback);
            }

            virtual void openDriver(const boost::filesystem::path &path
                                    , const OpenOptions &openOptions
                                    , const DriverOpenCallback::pointer
                                    &callback)
            {
                return owner->callback->openDriver
                    (path, openOptions, callback);
            }

            DoCallback::pointer owner;
            AggregatedDriver::DriverEntry &de;
        };

        virtual void done(Storage storage) {
            // fetch driver info
            const auto pd(prepareDrivers(storage, options));

            {
                // store info (under lock)
                std::unique_lock<std::mutex> guard(mutex);
                dependencies.storage = std::move(storage);
                dependencies.drivers = pd.drivers;
                expectDrivers = pd.drivers.size();
            }

            if (pd.drivers.empty()) {
                // no driver to open, we are finished here
                return done();
            }

            // we can move to the next phase: open all drivers to handle
            // this aggregated tileset

            // schedule driver open
            auto self(shared_from_this());
            auto idrivers(dependencies.drivers.begin());
            for (const auto &path : pd.paths) {
                // let the async machinery open the driver for us
                callback->openDriver
                    (path, openOptions
                     , std::make_shared<DriverOpener>(self, *idrivers));
                ++idrivers;
            }
        }

        virtual void error(const std::exception_ptr &exc) {
            callback->error(exc);
        }

        void newDriver() {
            // new driver fetched
            {
                std::unique_lock<std::mutex> guard(mutex);
                if (--expectDrivers) { return; }
            }

            done();
        }

        void done() {
            // last driver has been opened, we can finally construct this driver
            callback->done(std::make_shared<AggregatedDriver>
                           (AggregatedDriver::PrivateTag()
                            , root, openOptions, options, dependencies));
        }

        const boost::filesystem::path root;
        const OpenOptions openOptions;
        const AggregatedOptions options;
        const DriverOpenCallback::pointer callback;

        std::mutex mutex;
        AggregatedDriver::Dependencies dependencies;
        std::size_t expectDrivers;
    };

    // fire up the open machinery by opening storage first
    callback->openStorage
        (options.buildStoragePath(root)
         , std::make_shared<DoCallback>(root, openOptions, options, callback));
}

} } } // namespace vtslibs::vts::driver
