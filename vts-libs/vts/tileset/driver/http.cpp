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
#include "./http.hpp"

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

typedef HttpDriver::TileSetInfo TileSetInfo;
typedef TileSetInfo::GlueInfo GlueInfo;
typedef HttpDriver::EnhancedInfo EnhancedInfo;

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

} // namespace

HttpDriver::HttpDriver(const boost::filesystem::path &root
                                   , const HttpOptions &options
                                   , const CloneOptions &cloneOptions)
    : Driver(root, options, cloneOptions.mode())
{
    // TODO: fetch data from server
}

HttpDriver::HttpDriver(const boost::filesystem::path &root
                                   , const HttpOptions &options)
    : Driver(root, options)
{
    tileset::loadTileSetIndex(tsi_, *this);
}

HttpDriver::HttpDriver(const boost::filesystem::path &root
                                   , const HttpOptions &options
                                   , const CloneOptions &cloneOptions
                                   , const HttpDriver &src)
    : Driver(root, options, cloneOptions.mode())
{
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

    // and load it
    tileset::loadTileSetIndex(tsi_, *this);

    // make me read-only
    readOnly(true);
}

Driver::pointer
HttpDriver::clone_impl(const boost::filesystem::path &root
                             , const CloneOptions &cloneOptions)
    const
{
    return std::make_shared<HttpDriver>
        (root, options(), cloneOptions, *this);
}

HttpDriver::~HttpDriver() {}

OStream::pointer HttpDriver::output_impl(File type)
{
    if (readOnly()) {
        LOGTHROW(err2, storage::ReadOnlyError)
            << "This driver supports read access only.";
    }

    const auto path(root() / filePath(type));
    LOG(info1) << "Saving to " << path << ".";
    return fileOStream(type, path);
}

IStream::pointer HttpDriver::input_impl(File type) const
{
    auto path(root() / filePath(type));
    LOG(info1) << "Loading from " << path << ".";
    return fileIStream(type, path);
}

OStream::pointer HttpDriver::output_impl(const TileId&, TileFile)
{
    LOGTHROW(err2, storage::ReadOnlyError)
        << "This driver supports read access only.";
    return {};
}

IStream::pointer HttpDriver::input_impl(const TileId &tileId
                                        , TileFile type)
    const
{
    (void) tileId;
    (void) type;
    return {};
}

FileStat HttpDriver::stat_impl(File type) const
{
    const auto name(filePath(type));
    const auto path(root() / name);
    LOG(info1) << "Statting " << path << ".";
    return FileStat::stat(path);
}

FileStat HttpDriver::stat_impl(const TileId &tileId, TileFile type) const
{
    (void) tileId;
    (void) type;
    return {};
}

storage::Resources HttpDriver::resources_impl() const
{
    // nothing
    return {};
}


void HttpDriver::flush_impl() {
    LOGTHROW(err2, storage::ReadOnlyError)
        << "This driver supports read access only.";
}

void HttpDriver::drop_impl()
{
    LOGTHROW(err2, storage::ReadOnlyError)
        << "This driver supports read access only.";
}

} } } // namespace vadstena::vts::driver
