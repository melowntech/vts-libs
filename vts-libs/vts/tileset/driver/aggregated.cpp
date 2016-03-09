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

const std::string filePath(File type)
{
    switch (type) {
    case File::config: return ConfigName;
    case File::extraConfig: return ExtraConfigName;
    case File::tileIndex: return TileIndexName;
    default: break;
    }
    throw "unknown file type";
}

void unite(registry::IdSet &out, const registry::IdSet &in)
{
    out.insert(in.begin(), in.end());
}

void updateRanges(TileSet::Properties &out, const TileSet::Properties &in)
{
    // sanity check
    if (in.lodRange.empty()) { return; }

    // first hit
    if (out.lodRange.empty()) {
        out.lodRange = in.lodRange;
        out.tileRange = in.tileRange;
        return;
    }

    if (out.lodRange.min < in.lodRange.min) {
        // input is below current min lod

        // get tile ids for input
        auto min(tileId(in.lodRange.min, in.tileRange.ll));
        auto max(tileId(in.lodRange.min, in.tileRange.ur));

        // move input to current lod
        min = parent(min, (in.lodRange.min - out.lodRange.min));
        max = parent(max, (in.lodRange.min - out.lodRange.min));

        // output range plus re-lodded input range
        out.tileRange = unite
            (out.tileRange, TileRange(point(min), point(max)));
    } else if (in.lodRange.min < out.lodRange.min) {
        // input is above current min lod

        // get tile ids for output
        auto min(tileId(out.lodRange.min, out.tileRange.ll));
        auto max(tileId(out.lodRange.min, out.tileRange.ur));

        // move out to new lod
        min = parent(min, (out.lodRange.min - in.lodRange.min));
        max = parent(max, (out.lodRange.min - in.lodRange.min));

        // input range plus re-lodded output range
        out.tileRange = unite
            (TileRange(point(min), point(max)), in.tileRange);
    } else {
        out.tileRange = unite(out.tileRange, in.tileRange);
    }

    // unite ranges
    out.lodRange = unite(out.lodRange, in.lodRange);
}

typedef AggregatedDriver::TileSetInfo TileSetInfo;
typedef TileSetInfo::GlueInfo GlueInfo;
typedef AggregatedDriver::EnhancedInfo EnhancedInfo;

class StringIStream : public vs::IStream {
public:
    StringIStream(TileFile type, const std::string &name
                  , std::time_t lastModified)
        : vs::IStream(type), stat_(0, lastModified), name_(name)
    {}

    std::stringstream& sink() { return ss_; }

    void updateSize() { stat_.size = ss_.tellp(); }

private:
    virtual std::istream& get() { return ss_; }
    virtual vs::FileStat stat_impl() const { return stat_; }
    virtual void close() {};
    virtual std::string name() const { return name_; }

    vs::FileStat stat_;
    std::string name_;
    std::stringstream ss_;
};


IStream::pointer
buildMeta(const TileSetInfo::list &tsil, const fs::path &root
          , const registry::ReferenceFrame &referenceFrame
          , std::time_t lastModified, const TileId &tileId)
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
            if (gi.tsi.check(tileId, TileFile::meta)) {
                ometa.update(loadMeta(tileId, gi.driver), references, idx);
            }
        }

        // apply metatile from tileset
        if (tsi.tsi.check(tileId, TileFile::meta)) {
            ometa.update(loadMeta(tileId, tsi.driver), references, idx);
        }
    }

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
        LOG(debug) << "Trying set <" << info.name << ">.";
        if (info.tsi.real(tileId)) {
            return &info;
        }

        return nullptr;
    });

    typedef std::tuple<const EnhancedInfo*, int> GlueResult;

    auto tryGlues([&](const GlueInfo::list &glues) -> GlueResult
    {
        for (const auto &glue : glues) {
            if (const auto *result = trySet(glue)) {
                return GlueResult(result, 0);
            } if (auto reference = glue.tsi.getReference(tileId)) {
                LOG(debug)
                    << "Redirected to <" << glue.id[reference - 1]
                    << ">.";
                return GlueResult(nullptr, glue.indices[reference - 1] + 1);
            }
        }

        return GlueResult(nullptr, -1);
    });

    for (std::size_t idx(tsil.size()); idx;) {
        const auto &tsi(tsil[--idx]);

        // try glues first
        const EnhancedInfo *glueResult;
        int reference;
        std::tie(glueResult, reference) = tryGlues(tsi.glues);

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

/** Stores references that have to be erased from tile index before processing.
 *
 * TODO: make better -- this may still fail if there are two reference skips
 * for one tile like this:
 *
 *      A----------->D
 *          B--->C
 *
 * In this case we store result of C instead of D.
 *
 * Solution: single tile index which contains global tileset reference.
 * Obstacle: tile index can hold values of 8 bits only
 */
class ReferenceEraser {
public:
    void apply(int idx, TileIndex &ti) {
        LOG(info2) << "    applying tileset " << idx << ".";
        auto feraser(eraser_.find(idx));
        if (feraser == eraser_.end()) { return; }

        // erase filter
        auto filter([&](TileIndex::Flag::value_type value
                        , TileIndex::Flag::value_type erase)
                    -> TileIndex::Flag::value_type
        {
            if (erase && (value & TileIndex::Flag::reference)) {
                // tile marked as reference and should be removed here -> remove
                return 0;
            }

            // otherwise keep value
            return value;
        });

        ti.combine(feraser->second, filter);
        eraser_.erase(feraser);
    };

    void remember(const GlueInfo &gi) {
        for (int localIndex(1), le(gi.indices.size());
             localIndex <= le; ++localIndex)
        {
            auto globalIndex(gi.indices[localIndex - 1]);

            auto feraser(eraser_.find(globalIndex));
            if (feraser == eraser_.end()) {
                feraser = eraser_.insert(Eraser::value_type(globalIndex, {}))
                    .first;
            }

            // combine tile indices
            auto filter([&](TileIndex::Flag::value_type o
                            , TileIndex::Flag::value_type n)
                        -> TileIndex::Flag::value_type
            {
                return (o || (n == localIndex));
            });

            feraser->second.combine(gi.tsi.references, filter);

            LOG(info2) << "ReferenceEraser remember(<" << gi.name << ">): "
                       << localIndex << "(<"
                       << gi.id[localIndex - 1]
                       << ">) -> " << globalIndex << ": total count: "
                       << feraser->second.count() << ".";
        }
    }

private:
    typedef std::map<int, TileIndex> Eraser;
    Eraser eraser_;
};

} // namespace

AggregatedDriver::TileSetInfo::list
AggregatedDriver::buildTilesetInfo() const
{
    const auto &tilesets(this->options().tilesets);

    // grab tilesets and their glues
    TileSetGlues::list tilesetInfo;
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
    }

    TileSetInfo::list out;

    for (const auto &tsg : glueOrder(tilesetInfo)) {
        out.emplace_back(tsg);
        auto &tsi(out.back());

        // open tileset
        tsi.driver = Driver::open(storage_.path(tsi.tilesetId));
        tileset::loadTileSetIndex(tsi.tsi, *tsi.driver);
        tsi.name = tsi.tilesetId;

        LOG(debug) << "    <" << tsi.name << ">";

        // open glues
        for (auto &glue : tsi.glues) {
            glue.driver = Driver::open(storage_.path(glue));
            tileset::loadTileSetIndex(glue.tsi, *glue.driver);
            glue.name = boost::lexical_cast<std::string>
                (utility::join(glue.id, ","));
            LOG(debug) << "        <" << glue.name << ">";
        }
    }

    return out;
}

AggregatedDriver::AggregatedDriver(const boost::filesystem::path &root
                                   , const AggregatedOptions &options
                                   , CreateMode mode
                                   , const TilesetId &tilesetId)
    : Driver(root, options, mode)
    , storage_(this->options().storagePath, OpenMode::readOnly)
    , referenceFrame_(storage_.referenceFrame())
    , tilesetInfo_(buildTilesetInfo())
{
    TileSet::Properties properties;
    properties.id = tilesetId;
    properties.driverOptions = options;

    // try to get previous revision (and reuse)
    if (auto oldR = oldRevision()) {
        properties.revision = *oldR + 1;
    }

    // get reference frame
    properties.referenceFrame = storage_.getProperties().referenceFrame;
    properties.tileRange = TileRange(math::InvalidExtents{});
    properties.lodRange = LodRange::emptyRange();

    auto combiner([&](TileIndex::Flag::value_type o
                      , TileIndex::Flag::value_type n)
                  -> TileIndex::Flag::value_type
    {
        if (o & (TileIndex::Flag::real | TileIndex::Flag::reference)) {
            // already occupied by existing tile or non-removed reference
            return o;
        }

        // new tile data (whatever it is)
        return n;
    });

    // compose tile index and other properties
    TileIndex &ti(tsi_.tileIndex);
    ReferenceEraser eraser;

    bool first(true);
    for (std::size_t idx(tilesetInfo_.size()); idx; ) {
        const auto &tsg(tilesetInfo_[--idx]);
        const auto &tilesetId(tsg.tilesetId);
        LOG(info2) << "Adding tileset <" << tilesetId << ">.";

        // make room for referenced entities before any processing
        eraser.apply(idx, ti);

        for (const auto &glue : tsg.glues) {
            LOG(info2) << "    adding glue: " << glue.name;
            // remember references to be applied when referenced tileset is
            // processed
            eraser.remember(glue);
            ti.combine(glue.tsi.tileIndex, combiner);
        }

        const auto tsProp(tileset::loadConfig(*tsg.driver));
        ti.combine(tsg.tsi.tileIndex, combiner);

        // unite referenced registry entities
        unite(properties.credits, tsProp.credits);
        unite(properties.boundLayers, tsProp.boundLayers);

        // join various service data
        updateRanges(properties, tsProp);

        // TODO: spatial division extents

        // copy position from fist tileset
        if (first) {
            properties.position = tsProp.position;
            first = false;
        }
    }

    // remove any reference flag from tile index
    ti.unset(TileIndex::Flag::reference);

    // save stuff (allow write for a brief moment)
    readOnly(false);
    tileset::saveConfig(this->root() / filePath(File::config), properties);
    tileset::saveTileSetIndex(tsi_, *this);
    readOnly(true);
}

AggregatedDriver::AggregatedDriver(const boost::filesystem::path &root
                                   , const AggregatedOptions &options)
    : Driver(root, options)
    , storage_(this->options().storagePath, OpenMode::readOnly)
    , referenceFrame_(storage_.referenceFrame())
    , tilesetInfo_(buildTilesetInfo())
{
    tileset::loadTileSetIndex(tsi_, *this);
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

IStream::pointer AggregatedDriver::input_impl(File type) const
{
    auto path(root() / filePath(type));
    LOG(info1) << "Loading from " << path << ".";
    return fileIStream(type, path);
}

OStream::pointer AggregatedDriver::output_impl(const TileId&, TileFile)
{
    LOGTHROW(err2, storage::ReadOnlyError)
        << "This driver supports read access only.";
    return {};
}

IStream::pointer AggregatedDriver::input_impl(const TileId &tileId
                                              , TileFile type)
    const
{
    if (!tsi_.check(tileId, type)) {
        LOGTHROW(err1, vs::NoSuchFile)
            << "There is no " << type << " for " << tileId << ".";
    }

    if (type == TileFile::meta) {
        return buildMeta(tilesetInfo_, root(), referenceFrame_
                         , configStat().lastModified, tileId);
    }

    if (const auto *tsi = findTileSet(tilesetInfo_, tileId)) {
        return tsi->driver->input(tileId, type);
    }

    LOGTHROW(err1, vs::NoSuchFile)
        << "There is no " << type << " for " << tileId << ".";
    throw;
}

FileStat AggregatedDriver::stat_impl(File type) const
{
    (void) type;
    return {};
}

FileStat AggregatedDriver::stat_impl(const TileId &tileId, TileFile type) const
{
    (void) tileId;
    (void) type;
    return {};
}

storage::Resources AggregatedDriver::resources_impl() const
{
    return {};
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

} } } // namespace vadstena::vts::driver
