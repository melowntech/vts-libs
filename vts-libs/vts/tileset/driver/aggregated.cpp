/** Aggregated driver: on the fly surface aggregator
 *
 * TODO: cache stuff to make it a bit faster
 *
 * TODO: compose registry from all sub tilesets
 */

#include <stdexcept>
#include <limits>
#include <type_traits>
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

const std::string filePath(File type)
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

typedef AggregatedDriver::TileSetInfo TileSetInfo;
typedef TileSetInfo::GlueInfo GlueInfo;
typedef AggregatedDriver::EnhancedInfo EnhancedInfo;

IStream::pointer
buildMeta(const TileSetInfo::list &tsil, const fs::path &root
          , const registry::ReferenceFrame &referenceFrame
          , std::time_t lastModified, const TileId &tileId
          , const TileIndex &tileIndex, bool noSuchFile = true)
{
    // output metatile
    auto bo(referenceFrame.metaBinaryOrder);
    MetaTile ometa(tileId, bo);
    MetaTile::References references(ometa.makeReferences());

    auto loadMeta([&](const TileId &tileId, const Driver::pointer &driver)
                  -> MetaTile
    {
        auto ms(driver->input(tileId, TileFile::meta));
        return loadMetaTile(*ms, bo, ms->name());
    });

    // process whole input
    for (std::size_t idx(tsil.size()); idx; --idx) {
        const auto &tsi(tsil[idx - 1]);

        // process all glues
        for (const auto &gi : tsi.glues) {
            // apply metatile from glue
            if (gi.tsi->check(tileId, TileFile::meta)) {
                ometa.update(loadMeta(tileId, gi.driver), references
                             , idx, &gi.indices, gi.isAlien);
            }
        }

        // apply metatile from tileset
        if (tsi.tsi->check(tileId, TileFile::meta)) {
            ometa.update(loadMeta(tileId, tsi.driver), references, idx);
        }
    }

    if (ometa.empty()) {
        if (noSuchFile) {
            LOGTHROW(err1, vs::NoSuchFile)
                << "There is no metatile for " << tileId << ".";
        }
        return {};
    }

    // generate child flags based on tile index
    // TODO: make better by some quadtree magic
    ometa.for_each([&](const TileId &nodeId, MetaNode &node)
    {
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

const EnhancedInfo* findTileSet(const TileSetInfo::list &tsil
                                , const TileId &tileId)
{
#if 0
    typedef TileIndex::Flag TiFlag;
#endif

    auto trySet([&](const EnhancedInfo &info) -> const EnhancedInfo*
    {
        if (info.tsi->real(tileId, info.isAlien)) {
            // tile exists and has same alien flag
            LOG(debug)
                << "Tile " << tileId << " found in "
                << (info.isAlien ? "<(" : "<") << info.name
                << (info.isAlien ? ")>." : ">.");
            return &info;
        }

        LOG(debug)
            << "Missing tile " << tileId << " in "
            << (info.isAlien ? "<(" : "<") << info.name
            << (info.isAlien ? ")>." : ">.");
        return nullptr;
    });

    typedef std::tuple<const EnhancedInfo*, int> GlueResult;

    auto tryGlues([&](const GlueInfo::list &glues, int idx)
                  -> GlueResult
    {
        for (const auto &glue : glues) {
            if (const auto *result = trySet(glue)) {
                return GlueResult(result, 0);
            } if (auto reference = glue.tsi->getReference(tileId)) {
                auto redirect(glue.indices[reference - 1] + 1);
                if (redirect <= idx) {
                    LOG(debug)
                        << "Redirected to <" << glue.id[reference - 1]
                        << ">.";
                    return GlueResult(nullptr, redirect);
                }
                LOG(debug)
                    << "Unexpected forward reference <"
                    << glue.id[reference - 1] << ">.";
            }
        }

        return GlueResult(nullptr, -1);
    });

    for (std::size_t idx(tsil.size()); idx;) {
        const auto &tsi(tsil[--idx]);

        // try glues first
        const EnhancedInfo *glueResult;
        int reference;
        std::tie(glueResult, reference) = tryGlues(tsi.glues, idx);

        if (glueResult) {
            // found tile in one of glues
            return glueResult;
        } else if (reference > 0) {
            // found reference in one of glues -> redirect
            idx = reference;
        } else {
            // nothing found in glues, try tileset itself
            if (const auto *result = trySet(tsi)) {
                return result;
            }
        }
    }

    // nothing found
    return nullptr;
}

} // namespace

AggregatedDriver::TileSetInfo::list
AggregatedDriver::buildTilesetInfo() const
{
    LOG(info1) << "Building tileset info";
    // Step #1: grab all allowed tilesets and their glues
    const auto &tilesets(this->options().tilesets);

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

        // Step #2: distribute (possible) aliens in appropriet secondaty
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

    // Step #3: generate tileset info from nicely sorted glues of all tilesets;
    // aliens included
    TileSetInfo::list out;
    out.reserve(tilesetInfo.size());

    typedef std::map<TilesetId, TileSetInfo::GlueInfo> GlueMap;
    GlueMap glueMap;

    // process tileset
    for (const auto &tsg : glueOrder(tilesetInfo)) {
        out.emplace_back(referenceFrame_, tsg);
        auto &tsi(out.back());

        // open tileset
        tsi.driver = Driver::open(storage_.path(tsi.tilesetId));
        tileset::loadTileSetIndex(*tsi.tsi, *tsi.driver);
        tsi.name = tsi.tilesetId;

        LOG(info1) << "    <" << tsi.name << ">";

        // open glues
        for (auto iglues(tsi.glues.begin()); iglues != tsi.glues.end(); ) {
            auto &glue(*iglues);
            glue.name = boost::lexical_cast<std::string>
                (utility::join(glue.id, ","));

            auto fglueMap(glueMap.find(glue.name));
            if (fglueMap == glueMap.end()) {
                glue.driver = Driver::open(storage_.path(glue));
                tileset::loadTileSetIndex(*glue.tsi, *glue.driver);

                glue.hasAlienTiles = TileIndex::Flag::isAlien
                    (glue.tsi->tileIndex.allSetFlags());

                // cache
                glueMap.insert(GlueMap::value_type(glue.name, glue));
            } else {
                glue = fglueMap->second;
            }

            // mark as alien
            glue.isAlien = (glue.id.back() != tsg.tilesetId);

            if (glue.isAlien && !glue.hasAlienTiles) {
                // alien without alien tiles -> no need to have it here
                iglues = tsi.glues.erase(iglues);
            } else {
                LOG(info1) << "        <" << glue.name << ">"
                           << (glue.isAlien ? " (alien)" : "");
                ++iglues;
            }
        }
    }

    // done
    return out;
}

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
    , tilesetInfo_(buildTilesetInfo())
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
    , tilesetInfo_(buildTilesetInfo())
{
    // we flatten the content
    capabilities().flattener = true;

    // build driver information and cache it
    memProperties_ = build(options, cloneOptions);
}

TileSet::Properties AggregatedDriver::build(const AggregatedOptions &options
                                            , const CloneOptions &cloneOptions)
{
    TileSet::Properties properties;
    properties.id = *cloneOptions.tilesetId();
    properties.driverOptions = options;

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

    typedef TileIndex::Flag TiFlag;
    typedef TiFlag::value_type value_type;

    auto addFlags([](value_type &value, value_type flags)
    {
        // clear flags, keep reference (which is shared)
        value &= (0xffff0000u);
        value |= (flags & 0xff);
    });

    auto getFlags([](value_type value)
    {
        return (value & 0xff);
    });

    auto addIdx([](value_type &value, value_type idx)
    {
        value &= 0x0000ffffu;
        value |= (idx << 16);
    });

    auto getIdx([](value_type value)
    {
        return (value >> 16);
    });

    bool first(true);
    for (std::size_t idx(tilesetInfo_.size()); idx; --idx) {
        // marks alien tileset
        bool alien(false);

        auto combiner([&](value_type o, value_type n) -> value_type
        {
            if (o & TiFlag::mesh) {
                // already occupied by existing tile
                return o;
            }

            if (auto reference = getIdx(o)) {
                // we have valid reference here
                if (reference != idx) {
                    return o;
                }
            }

            // store flags only if node is of same type
            if (alien == TiFlag::isAlien(n)) {
                addFlags(o, n);
            }
            return o;
        });

        const auto &tsg(tilesetInfo_[idx - 1]);
        const auto &tilesetId(tsg.tilesetId);
        LOG(info2) << "Adding tileset <" << tilesetId << "> (" << idx << ").";

        for (const auto &glue : tsg.glues) {
            LOG(info2) << "    adding glue: " << glue.name;
            // remember references to be applied when referenced tileset is
            // processed
            auto storeReferences([&](value_type o, value_type n) -> value_type
            {
                // valid reference + no mesh tile stored + no reference marked
                // -> mark reference
                if (n && !(o & TileIndex::Flag::mesh) && !getIdx(o)) {
                    addIdx(o, glue.indices[n - 1] + 1);
                }
                return o;
            });

            ti.combine(glue.tsi->references, storeReferences);
            alien = glue.isAlien;
            ti.combine(glue.tsi->tileIndex, combiner);
        }

        const auto tsProp(tileset::loadConfig(*tsg.driver));
        alien = false;
        ti.combine(tsg.tsi->tileIndex, combiner);

        // unite referenced registry entities
        unite(properties.credits, tsProp.credits);
        unite(properties.boundLayers, tsProp.boundLayers);

        // TODO: spatial division extents

        // copy position from first tileset
        if (first) {
            properties.position = tsProp.position;
            first = false;
        }
    }

    // remove nonsense flags
    ti.unset(TiFlag::reference | 0xffff0000u);

    // update extents
    {
        auto ranges(ti.ranges(TiFlag::mesh | TiFlag::reference));
        properties.lodRange = ranges.first;
        properties.tileRange = ranges.second;
    }

    return properties;
}

AggregatedDriver::AggregatedDriver(const boost::filesystem::path &root
                                   , const AggregatedOptions &options)
    : Driver(root, options)
    , storage_(this->options().storagePath, OpenMode::readOnly)
    , referenceFrame_(storage_.referenceFrame())
    , tsi_(referenceFrame_.metaBinaryOrder)
    , tilesetInfo_(buildTilesetInfo())
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
    , tilesetInfo_(buildTilesetInfo())
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

    if (!tsi_.check(tileId, type)) {
        if (noSuchFile) {
            LOGTHROW(err1, vs::NoSuchFile)
                << "There is no " << type << " for " << tileId << ".";
        }
        return {};
    }

    if (type == TileFile::meta) {
        return buildMeta(tilesetInfo_, root(), referenceFrame_
                         , configStat().lastModified, tileId
                         , tsi_.tileIndex, noSuchFile);
    }

    if (const auto *tsi = findTileSet(tilesetInfo_, tileId)) {
        return tsi->driver->input(tileId, type);
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

    LOGTHROW(err1, vs::NoSuchFile)
        << "There is no " << type << " for " << tileId << ".";
    throw;
}

storage::Resources AggregatedDriver::resources_impl() const
{
    // accumulate resources
    storage::Resources resources;
    for (const auto &tsi : tilesetInfo_) {
        resources += tsi.driver->resources();
        for (const auto &glue : tsi.glues) {
            resources += glue.driver->resources();
        }
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
    os << "aggregated (storage=" << o.storagePath
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
