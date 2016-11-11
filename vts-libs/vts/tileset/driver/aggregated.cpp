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
#include "utility/base64.hpp"

#include "../../../storage/error.hpp"
#include "../../../storage/fstreams.hpp"
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

    return utility::base64::encode(os.str());
}

inline AggregatedDriver::TilesetReferencesList
deserializeTsMap(const std::string &raw)
{
    using utility::binaryio::read;
    std::istringstream is(utility::base64::decode(raw));
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

AggregatedDriver::DriverEntry::list
openDrivers(Storage &storage, const AggregatedOptions &options)
{
    // get list of tilesets (in proper order)
    const auto tilesets(storage.tilesets(options.tilesets));

    const auto tsMap(deserializeTsMap(options.tsMap));
    AggregatedDriver::DriverEntry::list drivers;

    drivers.reserve(tsMap.size());

    for (const auto &references : tsMap) {
        // construct glue id
        Glue::Id glueId;
        for (auto reference : references) {
            glueId.push_back(tilesets[reference]);
        }

        if (glueId.size() == 1) {
            // tileset
            LOG(info4) << "opening <" << glueId.front() << ">";
            drivers.emplace_back(Driver::open(storage.path(glueId.front()))
                                 , references);
        } else {
            // glue
            LOG(info4) << "opening <" << utility::join(glueId, ",") << ">";
            drivers.emplace_back(Driver::open(storage.path(glueId))
                                 , references);
        }
    }

    return drivers;
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
    , storage_(this->options().storagePath, OpenMode::readOnly)
    , referenceFrame_(storage_.referenceFrame())
    , tsi_(referenceFrame_.metaBinaryOrder)
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
    , tsi_(referenceFrame_.metaBinaryOrder)
{
    // we flatten the content
    capabilities().flattener = true;

    // build driver information and cache it
    memProperties_ = build(options, cloneOptions);
}

namespace {

typedef TileIndex::Flag TiFlag;
typedef TiFlag::value_type value_type;
typedef std::uint16_t SetId;

inline SetId setIdFromFlags(TileIndex::Flag::value_type flags)
{
    return flags >> 16;
}

void addTileIndex(TileIndex &ti, const Driver::pointer &driver
                  , SetId setId, bool alien)
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

        // combine flags with set id
        return (n & TiFlag::nonAlien) | (setId << 16);
    });

    // process tileindex
    tileset::Index work;
    tileset::loadTileSetIndex(work, *driver);
    ti.combine(work.tileIndex, combiner);
}

} // namespace

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
    DriverEntry::list drivers;
    drivers.reserve(setCount);

    typedef std::map<Glue::Id, DriverEntry*> Glue2Driver;
    Glue2Driver glue2driver;

    TilesetReferencesList tsMap;

    for (const auto &tsg : go) {
        LOG(info4) << "<" << tsg.tilesetId << ">";

        // first, remember tileset
        drivers.emplace_back
            (Driver::open(storage_.path(tsg.tilesetId))
             , tileset2references(tsg.tilesetId));
        auto &de(drivers.back());
        const auto setId(drivers.size() - 1);
        tsMap.push_back(de.tilesets);

        // then process all glues
        for (const auto &glue : tsg.glues) {
            bool alien(glue.id.back() != tsg.tilesetId);
            DriverEntry *de(nullptr);

            if (!alien) {
                drivers.emplace_back
                    (Driver::open(storage_.path(glue))
                     , glue2references(glue.id));

                de = &drivers.back();
                glue2driver[glue.id] = de;
                tsMap.push_back(de->tilesets);

                LOG(info4)
                    << "    <" << utility::join(glue.id, ",") << "> ("
                    << de->driver << ")";
            } else {
                de = glue2driver.at(glue.id);
                LOG(info4)
                    << "   *<" << utility::join(glue.id, ",") << "> ("
                    << de->driver << ")";
            }

            // merge-in glue's tile index
            addTileIndex(ti, de->driver, drivers.size() - 1, alien);
        }

        // and merge-in tileset's tile index last
        addTileIndex(ti, de.driver, setId, false);
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
                                   , const AggregatedOptions &options)
    : Driver(root, options)
    , storage_(this->options().storagePath, OpenMode::readOnly)
    , referenceFrame_(storage_.referenceFrame())
    , drivers_(openDrivers(storage_, options))
    , tsi_(referenceFrame_.metaBinaryOrder)
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
    , storage_(this->options().storagePath, OpenMode::readOnly)
    , referenceFrame_(storage_.referenceFrame())
    , tsi_(referenceFrame_.metaBinaryOrder)
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

    drivers_ = openDrivers(storage_, options);

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
    if (noSuchFile) {
        return fileIStream(type, path);
    }
    return fileIStream(type, path, NullWhenNotFound);
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
    // if (!tsi_.check(tileId, type)) {
    //     if (noSuchFile) {
    //         LOGTHROW(err1, vs::NoSuchFile)
    //             << "There is no " << type << " for " << tileId << ".";
    //     }
    //     return {};
    // }

    if (type == TileFile::meta) {
        if (!tsi_.meta(tileId)) {
            if (noSuchFile) {
                LOGTHROW(err1, vs::NoSuchFile)
                    << "There is no " << type << " for " << tileId << ".";
            }
            return {};
        }

#if 0
        return buildMeta(tilesetInfo_, root(), referenceFrame_
                         , configStat().lastModified, tileId
                         , tsi_.tileIndex, noSuchFile);
#endif
    }

    const auto flags(tsi_.checkAndGetFlags(tileId, type));
    if (!flags) {
        if (noSuchFile) {
            LOGTHROW(err1, vs::NoSuchFile)
                << "There is no " << type << " for " << tileId << ".";
        }
        return {};
    }

    // get dataset id
    const auto setId(setIdFromFlags(flags));

    if (setId < drivers_.size()) {
        return drivers_[setId].driver->input(tileId, type);
    }

    if (noSuchFile) {
        LOGTHROW(err1, vs::NoSuchFile)
            << "There is no " << type << " for " << tileId << ".";
    }

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
#if 0
    if (!tsi_.check(tileId, type)) {
        LOGTHROW(err1, vs::NoSuchFile)
            << "There is no " << type << " for " << tileId << ".";
    }

    if (type == TileFile::meta) {
        // TODO: make better
        return buildMeta(tilesetInfo_, root(), referenceFrame_
                         , configStat().lastModified, tileId
                         , tsi_.tileIndex)->stat();
    }

    if (const auto *tsi = findTileSet(tilesetInfo_, tileId)) {
        return tsi->driver->stat(tileId, type);
    }
#endif

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
       << ", tilesets=[" << utility::join(o.tilesets, " ") << "])";
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

} } } // namespace vadstena::vts::driver
