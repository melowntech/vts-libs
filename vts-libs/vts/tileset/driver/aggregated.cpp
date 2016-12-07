/** Aggregated driver: on the fly surface aggregator
 *
 * TODO: cache stuff to make it a bit faster
 *
 * TODO: compose registry from all sub tilesets
 */

#include <stdexcept>
#include <limits>
#include <type_traits>
#include <algorithm>
#include <fstream>

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

#include "../../../storage/error.hpp"
#include "../../../storage/fstreams.hpp"
#include "../../../storage/sstreams.hpp"
#include "../../../storage/io.hpp"
#include "../../io.hpp"
#include "../../tileflags.hpp"
#include "../config.hpp"
#include "../detail.hpp"
#include "./aggregated.hpp"

namespace vadstena { namespace vts { namespace driver {

namespace fs = boost::filesystem;

namespace vs = vadstena::storage;

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

const char TM_MAGIC[2] = { 'T', 'M' };

inline std::string
serializeTsMap(const AggregatedDriver::TilesetReferencesList &tsMap)
{
    using utility::binaryio::write;
    std::ostringstream os;
    os.exceptions(std::ostream::failbit | std::ostream::badbit);

    write(os, TM_MAGIC, sizeof(TM_MAGIC));

    // write number of datasets
    write(os, std::uint16_t(tsMap.size()));

    // write all references
    for (const auto &references : tsMap) {
        write(os, std::uint8_t(references.size()));
        for (auto reference : references) {
            write(os, reference);
        }
    }

    return os.str();
}

inline AggregatedDriver::TilesetReferencesList
deserializeTsMap(const std::string &raw)
{
    using utility::binaryio::read;
    std::istringstream is(raw);
    is.exceptions(std::istream::failbit | std::istream::badbit);

    char magic[sizeof(TM_MAGIC)];
    read(is, magic);
    if (std::memcmp(magic, TM_MAGIC, sizeof(TM_MAGIC))) {
        LOGTHROW(err1, storage::BadFileFormat)
            << "Invalid tile mapping magic.";
    }

    std::uint16_t mapCount;
    read(is, mapCount);

    AggregatedDriver::TilesetReferencesList tsMap;
    tsMap.resize(mapCount);

    for (auto &references : tsMap) {
        std::uint8_t rCount;
        read(is, rCount);
        references.resize(rCount);

        for (auto &reference : references) {
            read(is, reference);
        }
    }

    return tsMap;
}

typedef TileIndex::Flag TiFlag;
typedef TiFlag::value_type value_type;

inline MetaNode::SourceReference
sourceReferenceFromFlags(TileIndex::Flag::value_type flags)
{
    return flags >> 16;
}

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
        de.metaIndex = work.deriveMetaIndex();
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

    auto loadMeta([&](const TileId &tileId, const Driver::pointer &driver)
                  -> MetaTile
    {
        auto ms(noSuchFile
                ? driver->input(tileId, TileFile::meta)
                : driver->input(tileId, TileFile::meta, NullWhenNotFound));
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
    // TODO: make better by some quadtree magic
    ometa.for_each([&](const TileId &nodeId, MetaNode &node)
    {
        if (!keepSurfaceReferences) {
            // do not keep surface references -> reset
            node.sourceReference = 0;
        }
        for (const auto &child : vts::children(nodeId)) {
            node.setChildFromId
                (child, tileIndex.validSubtree(child));
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

} // namespace

AggregatedDriverBase::AggregatedDriverBase(const CloneOptions &cloneOptions)
{
    if (!cloneOptions.tilesetId()) {
        LOGTHROW(err2, storage::NoSuchTileSet)
            << "Attempt to create aggregated driver without providing "
            "tilesetId.";
    }
}

AggregatedDriver::AggregatedDriver(const boost::filesystem::path &root
                                   , const AggregatedOptions &options
                                   , const CloneOptions &cloneOptions)
    : AggregatedDriverBase(cloneOptions)
    , Driver(root, options, cloneOptions.mode())
    , storage_(fs::absolute(this->options().storagePath, root)
               , OpenMode::readOnly)
    , referenceFrame_(storage_.referenceFrame())
    , tsi_(referenceFrame_.metaBinaryOrder, drivers_)
    , surfaceReferences_(this->options().surfaceReferences)
{
    // we flatten the content
    capabilities().flattener = true;

    // build driver information
    auto properties(build(options, cloneOptions));

    // save stuff (allow write for a brief moment)
    tileset::saveConfig(this->root() / filePath(File::config), properties);
    tileset::saveTileSetIndex(tsi_, *this);

    readOnly(true);
}

AggregatedDriver::AggregatedDriver(const AggregatedOptions &options
                                   , const CloneOptions &cloneOptions)
    : AggregatedDriverBase(cloneOptions)
    , Driver(options, cloneOptions.mode())
    , storage_(this->options().storagePath, OpenMode::readOnly)
    , referenceFrame_(storage_.referenceFrame())
    , tsi_(referenceFrame_.metaBinaryOrder, drivers_)
    , surfaceReferences_(this->options().surfaceReferences)
{
    // we flatten the content
    capabilities().flattener = true;

    // build driver information and cache it
    memProperties_ = build(options, cloneOptions);
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
                                            , const CloneOptions &cloneOptions)
{
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

    // write options
    properties.driverOptions = options;
    return properties;
}

AggregatedDriver::AggregatedDriver(const boost::filesystem::path &root
                                   , const OpenOptions &openOptions
                                   , const AggregatedOptions &options)
    : Driver(root, openOptions, options)
    , storage_(fs::absolute(this->options().storagePath, root)
               , OpenMode::readOnly)
    , referenceFrame_(storage_.referenceFrame())
    , drivers_(openDrivers(storage_, openOptions, options))
    , tsi_(referenceFrame_.metaBinaryOrder, drivers_)
    , surfaceReferences_(this->options().surfaceReferences)
{
    // we flatten the content
    capabilities().flattener = true;

    tileset::loadTileSetIndex(tsi_, *this);
}

AggregatedDriver::AggregatedDriver(const boost::filesystem::path &root
                                   , const AggregatedOptions &options
                                   , const CloneOptions &cloneOptions
                                   , const AggregatedDriver &src)
    : Driver(root, options, cloneOptions.mode())
    , storage_(fs::absolute(this->options().storagePath, root)
               , OpenMode::readOnly)
    , referenceFrame_(storage_.referenceFrame())
    , tsi_(referenceFrame_.metaBinaryOrder, drivers_)
    , surfaceReferences_(this->options().surfaceReferences)
{
    // we flatten the content
    capabilities().flattener = true;

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

    // clone tile index
    copyFile(src.input(File::tileIndex), output(File::tileIndex));

    drivers_ = openDrivers(storage_, cloneOptions.openOptions(), options);

    // and load it
    tileset::loadTileSetIndex(tsi_, *this);

    // make me read-only
    readOnly(true);
}

Driver::pointer
AggregatedDriver::clone_impl(const boost::filesystem::path &root
                             , const CloneOptions &cloneOptions)
    const
{
    return std::make_shared<AggregatedDriver>
        (root, options(), cloneOptions, *this);
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

        return buildMeta(drivers_, root(), referenceFrame_
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

        return buildMeta(drivers_, root(), referenceFrame_
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

} } } // namespace vadstena::vts::driver
