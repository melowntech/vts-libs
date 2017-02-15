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
#include "./aggregated-old.hpp"

namespace vtslibs { namespace vts { namespace driver {

namespace fs = boost::filesystem;

namespace vs = vtslibs::storage;

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

typedef OldAggregatedDriver::TileSetInfo TileSetInfo;
typedef TileSetInfo::GlueInfo GlueInfo;
typedef OldAggregatedDriver::EnhancedInfo EnhancedInfo;

IStream::pointer
buildMeta(const TileSetInfo::list &tsil, const fs::path &root
          , const registry::ReferenceFrame &referenceFrame
          , std::time_t lastModified, const TileId &tileId
          , const TileIndex &tileIndex, bool noSuchFile = true)
{
    // output metatile
    auto bo(referenceFrame.metaBinaryOrder);
    MetaTile ometa(tileId, bo);

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
                ometa.update(loadMeta(tileId, gi.driver), gi.isAlien);
            }
        }

        // apply metatile from tileset
        if (tsi.tsi->check(tileId, TileFile::meta)) {
            ometa.update(loadMeta(tileId, tsi.driver));
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

    auto tryGlues([&](const GlueInfo::list &glues) -> const EnhancedInfo*
    {
        for (const auto &glue : glues) {
            if (const auto *result = trySet(glue)) {
                return result;
            }
        }

        return nullptr;
    });

    for (std::size_t idx(tsil.size()); idx;) {
        const auto &tsi(tsil[--idx]);

        // try glues first
        if (auto glueResult = tryGlues(tsi.glues)) {
            // found tile in one of glues
            return glueResult;
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

OldAggregatedDriver::TileSetInfo::list
OldAggregatedDriver::buildTilesetInfo() const
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
        tsi.driver = Driver::open
            (storage_.path(tsi.tilesetId), openOptions());
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
                glue.driver = Driver::open
                    (storage_.path(glue), openOptions());
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

OldAggregatedDriverBase::OldAggregatedDriverBase(const CloneOptions &cloneOptions)
{
    if (!cloneOptions.tilesetId()) {
        LOGTHROW(err2, storage::NoSuchTileSet)
            << "Attempt to create aggregated driver without providing "
            "tilesetId.";
    }
}

OldAggregatedDriver::OldAggregatedDriver(const boost::filesystem::path &root
                                         , const OpenOptions &openOptions
                                         , const OldAggregatedOptions &options)
    : Driver(root, openOptions, options)
    , storage_(this->options().storagePath, OpenMode::readOnly)
    , referenceFrame_(storage_.referenceFrame())
    , tsi_(referenceFrame_.metaBinaryOrder)
    , tilesetInfo_(buildTilesetInfo())
{
    // we flatten the content
    capabilities().flattener = true;

    tileset::loadTileSetIndex(tsi_, *this);
}

OldAggregatedDriver::OldAggregatedDriver(const boost::filesystem::path &root
                                   , const OldAggregatedOptions &options
                                   , const CloneOptions &cloneOptions
                                   , const OldAggregatedDriver &src)
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
OldAggregatedDriver::clone_impl(const boost::filesystem::path &root
                             , const CloneOptions &cloneOptions)
    const
{
    return std::make_shared<OldAggregatedDriver>
        (root, options(), cloneOptions, *this);
}

OldAggregatedDriver::~OldAggregatedDriver() {}

OStream::pointer OldAggregatedDriver::output_impl(File type)
{
    if (readOnly()) {
        LOGTHROW(err2, storage::ReadOnlyError)
            << "This driver supports read access only.";
    }

    const auto path(root() / filePath(type));
    LOG(info1) << "Saving to " << path << ".";
    return fileOStream(type, path);
}

IStream::pointer OldAggregatedDriver::input_mem(File type) const
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

IStream::pointer OldAggregatedDriver::input_impl(File type, bool noSuchFile)
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

OStream::pointer OldAggregatedDriver::output_impl(const TileId&, TileFile)
{
    LOGTHROW(err2, storage::ReadOnlyError)
        << "This driver supports read access only.";
    return {};
}

IStream::pointer OldAggregatedDriver::input_impl(const TileId &tileId
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

FileStat OldAggregatedDriver::stat_impl(File type) const
{
    const auto name(filePath(type));
    const auto path(root() / name);
    LOG(info1) << "Statting " << path << ".";
    return FileStat::stat(path);
}

FileStat OldAggregatedDriver::stat_impl(const TileId &tileId, TileFile type) const
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

storage::Resources OldAggregatedDriver::resources_impl() const
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

void OldAggregatedDriver::flush_impl() {
    LOGTHROW(err2, storage::ReadOnlyError)
        << "This driver supports read access only.";
}

void OldAggregatedDriver::drop_impl()
{
    LOGTHROW(err2, storage::ReadOnlyError)
        << "This driver supports read access only.";
}

std::string OldAggregatedDriver::info_impl() const
{
    auto o(options());
    std::ostringstream os;
    os << "aggregated (storage=" << o.storagePath
       << ", tilesets=[" << utility::join(o.tilesets, " ") << "])";
    return os.str();
}

boost::any OldAggregatedOptions::relocate(const RelocateOptions &options
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

} } } // namespace vtslibs::vts::driver
