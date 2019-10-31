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

#include <unistd.h>

#include <cerrno>
#include <queue>

#include <boost/format.hpp>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/predicate.hpp>
#include <boost/preprocessor/seq.hpp>
#include <boost/preprocessor/stringize.hpp>

#include <opencv2/highgui/highgui.hpp>

#include "dbglog/dbglog.hpp"

#include "utility/enum-io.hpp"
#include "utility/gccversion.hpp"
#include "utility/streams.hpp"
#include "utility/time.hpp"
#include "utility/filedes.hpp"
#include "utility/implicit-value.hpp"

#include "service/cmdline.hpp"
#include "service/runninguntilsignalled.cpp"

#include "math/io.hpp"
#include "math/transform.hpp"

#include "imgproc/png.hpp"
#include "imgproc/rastermask/cvmat.hpp"

#include "geo/geodataset.hpp"

#include "../registry/po.hpp"
#include "../registry/urlexpander.hpp"
#include "../vts.hpp"
#include "../vts/io.hpp"
#include "../vts/mesh.hpp"
#include "../vts/atlas.hpp"
#include "../vts/tileflags.hpp"
#include "../vts/metaflags.hpp"
#include "../vts/encodeflags.hpp"
#include "../vts/opencv/colors.hpp"
#include "../vts/opencv/navtile.hpp"
#include "../vts/tileset/delivery.hpp"
#include "../vts/tileset/driver.hpp"
#include "../vts/2d.hpp"
#include "../vts/visit.hpp"
#include "../vts/csconvertor.hpp"
#include "../vts/options-po.hpp"
#include "../vts/meshopinput.hpp"
#include "../vts/tsmap.hpp"

#include "locker.hpp"
#include "support/urlfetcher.hpp"

namespace po = boost::program_options;
namespace fs = boost::filesystem;
namespace ba = boost::algorithm;
namespace vts = vtslibs::vts;
namespace vr = vtslibs::registry;
namespace vs = vtslibs::storage;
namespace tools = vtslibs::vts::tools;

#define VTS_COMMAND_LIST                                            \
    ((info))                                                        \
    ((save))                                                        \
    ((create))                                                      \
    ((add))                                                         \
    ((remove))                                                      \
    ((generateGlues)("glue-generate-pending"))                      \
    ((generateGlue)("glue-generate"))                               \
    ((listPendingGlues)("list-pending-glues"))                      \
                                                                    \
    ((dumpMetatile)("dump-metatile"))                               \
    ((dumpMetatileFile)("dump-metatile-file"))                      \
    ((metatileVersion)("metatile-version"))                         \
    ((mapConfig)("map-config"))                                     \
    ((dirs)("dirs"))                                                \
    ((dumpTileIndex)("dump-tileindex"))                             \
    ((tileInfo)("tile-info"))                                       \
    ((tileStatus)("tile-status"))                                   \
    ((dumpMesh)("dump-mesh"))                                       \
    ((dumpMeshMask)("dump-mesh-mask"))                              \
    ((exportMesh)("export-mesh"))                                   \
    ((tileMultifileLayout)("tile-multifile-layout"))                \
    ((tileIndexInfo)("tileindex-info"))                             \
    ((tileIndexRanges)("tileindex-ranges"))                         \
    ((convertTileIndex)("convert-tileindex"))                       \
    ((concat)("concat"))                                            \
    ((aggregate)("aggregate"))                                      \
    ((remote)("remote"))                                            \
    ((local)("local"))                                              \
    ((clone)("clone"))                                              \
    ((relocate)("relocate"))                                        \
    ((reencode)("reencode"))                                        \
    ((reencodeCleanup)("reencode-cleanup"))                         \
    ((tilePick)("tile-pick"))                                       \
    ((file)("file"))                                                \
    ((tags)("tags"))                                                \
    ((proxies)("proxies"))                                          \
    ((glueRulesSyntax)("glue-rules-syntax"))                        \
    ((mergeConfSyntax)("merge-conf-syntax"))                        \
    ((dumpNavtile)("dump-navtile"))                                 \
    ((dumpNavtileMask)("dump-navtile-mask"))                        \
    ((navtile2dem))                                                 \
    ((queryNavtile)("query-navtile"))                               \
    ((showLockerApi)("show-locker-api"))                            \
    ((deriveMetaIndex)("derive-metaindex"))                         \
    ((virtualSurfaceCreate)("vs-create"))                           \
    ((virtualSurfaceRemove)("vs-remove"))                           \
    ((virtualSurfaceInfo)("vs-info"))                               \
                                                                    \
    ((checkMetatileTree)("check-metatile-tree"))                   \
                                                                    \
    ((locker2Stresser)("locker2-stresser"))                         \
                                                                    \
    ((decodeTsMap)("decode-tsmap"))                                 \
                                                                    \
    ((listReferenceFrames)("list-reference-frames"))

UTILITY_GENERATE_ENUM(Command, VTS_COMMAND_LIST)

struct Verbosity {
    int level;
    Verbosity(int level = 0) : level(level) {}
    operator int() const { return level; }
};

void validate(boost::any &v, const std::vector<std::string>&, Verbosity*, int)
{
    if (v.empty()) {
        v = Verbosity(1);
    } else {
        ++boost::any_cast<Verbosity&>(v).level;
    }
}

struct MetaLodRange {
    vts::LodRange lodRange;
};

typedef std::vector<std::string> Values;

vts::Tags tagsFromValues(const Values &values)
{
    return { values.begin(), values.end() };
}

vts::Proxy2ExternalUrl proxiesFromValues(const Values &values)
{
    vts::Proxy2ExternalUrl mapping;

    for (const auto &value : values) {
        const auto eq(value.find('='));
        if (eq == std::string::npos) { continue; }
        mapping[value.substr(0, eq)] = value.substr(eq + 1);
    }

    return mapping;
}

UTILITY_GENERATE_ENUM(MeshFormat,
                      ((geo))
                      ((normalized))
                      ((sds))
                      ((sdsNormalized)("sds-normalized"))
                      ((coverage))
                      )

void validate(boost::any &v, const std::vector<std::string> &values
              , MetaLodRange*, int)
{
    po::validators::check_first_occurrence(v);
    const auto &s(po::validators::get_single_string(values));

    if (s == "full") {
        v = MetaLodRange{vts::LodRange::maxRange()};
        return;
    }

    v = MetaLodRange{boost::lexical_cast<vts::LodRange>(s)};
}

typedef service::UnrecognizedParser UP;

class VtsStorage : public service::Cmdline {
public:
    VtsStorage()
        : service::Cmdline("vts", BUILD_TARGET_VERSION
                           , (service::DISABLE_EXCESSIVE_LOGGING
                              | service::ENABLE_UNRECOGNIZED_OPTIONS))
        , noexcept_(false), command_(Command::info)
        , tileFlags_(), metaFlags_(), encodeFlags_()
        , queryLod_(), textureQuality_(70), meshFormat_(MeshFormat::normalized)
        , generate_(false), sameType_(false)
        , timeout_(5000)
    {
        addOptions_.textureQuality = 0;
        addOptions_.checkTileindexIdentity = true;
        addOptions_.bumpVersion = false;
        addOptions_.dryRun = false;
        addOptions_.mode = vts::Storage::AddOptions::Mode::legacy;
        addOptions_.glueMode = vts::GlueMode::coverageContour;
        addOptions_.collisionCheck = true;
        addOptions_.skirtMode = vts::SkirtMode::none;
        addOptions_.skirtScale = 1.0;

        relocateOptions_.dryRun = false;
        reencodeOptions_.dryRun = false;
        reencodeOptions_.cleanup = false;
    }

    ~VtsStorage() {}

private:
    virtual void configuration(po::options_description &cmdline
                               , po::options_description &config
                               , po::positional_options_description &pd)
        UTILITY_OVERRIDE;

    virtual void configure(const po::variables_map &vars)
        UTILITY_OVERRIDE;

    virtual UP::optional
    configure(const po::variables_map &vars
              , const std::vector<std::string> &unrecognized)
        UTILITY_OVERRIDE;

    virtual po::ext_parser extraParser() UTILITY_OVERRIDE;

    virtual std::vector<std::string> listHelps() const UTILITY_OVERRIDE;

    virtual bool help(std::ostream &out, const std::string &what) const
        UTILITY_OVERRIDE;

    virtual int run() UTILITY_OVERRIDE;

    template<typename Body>
    void createParser(po::options_description &cmdline, Command command
                      , const std::string &help, Body body)
    {
        auto p(std::make_shared<UP>(help));
        body(*p);
        commandParsers_[command] = p;

        auto name(boost::lexical_cast<std::string>(command));
        cmdline.add_options()
            (name.c_str(), ("alias for --command=" + name).c_str())
            ;
    }

    UP::optional getParser(Command command)
        const;

    void lockConfiguration(po::options_description &config);

    void lockConfigure(const po::variables_map &vars);

    int runCommand();

    int info();

    int save();

    int create();

    int add();

    int remove();

    int generateGlues();

    int generateGlue();

    int listPendingGlues();

    int tags();

    int proxies();

    int dumpMetatile();

    int dumpMetatileFile();

    int metatileVersion();

    int mapConfig();

    int dirs();

    int dumpTileIndex();

    int tileInfo();
    int tileStatus();

    int dumpMesh();
    int dumpMeshMask();
    int exportMesh();

    int tileMultifileLayout();

    int tileIndexInfo();
    int tileIndexRanges();

    int convertTileIndex();

    int concat();

    int aggregate();

    int remote();

    int local();

    int clone();

    int relocate();

    int reencode();
    int reencodeCleanup();

    int tilePick();

    int file();

    int glueRulesSyntax();
    int mergeConfSyntax();

    int dumpNavtile();
    int dumpNavtileMask();
    int navtile2dem();

    int showLockerApi();

    int deriveMetaIndex();

    int virtualSurfaceCreate();
    int virtualSurfaceRemove();
    int virtualSurfaceInfo();

    int checkMetatileTree();

    int queryNavtile();

    int locker2Stresser();

    int decodeTsMap();

    int listReferenceFrames();

    bool noexcept_;
    fs::path path_;
    Command command_;

    vts::CreateMode createMode_;
    vts::StorageProperties storageProperties_;
    vr::ReferenceFrame referenceFrame_;
    vts::TileId tileId_;
    fs::path tileset_;
    std::vector<fs::path> tilesets_;
    std::string tilesetId_;
    std::vector<std::string> tilesetIds_;
    boost::optional<std::string> optTilesetId_;
    vts::Storage::Location where_;
    boost::optional<vts::LodRange> optLodRange_;
    fs::path outputPath_;
    vts::TileFlags tileFlags_;
    vts::MetaFlags metaFlags_;
    vts::EncodeFlags encodeFlags_;
    std::vector<vts::TileId> tileIds_;
    std::string remoteUrl_;
    fs::path localPath_;
    boost::optional<std::string> optSrs_;
    vts::Storage::AddOptions addOptions_;
    vts::RelocateOptions relocateOptions_;
    vts::ReencodeOptions reencodeOptions_;
    Verbosity verbose_;
    bool computeTexelSize_;
    vs::CreditIds forceCredits_;
    Values addValues_;
    Values removeValues_;

    boost::optional<vts::Lod> queryLod_;
    math::Point2IOWrapper<double> queryPoint_;

    int textureQuality_;

    MeshFormat meshFormat_;

    bool generate_;

    bool sameType_;

    vts::MapConfigOptions mco_;

    boost::optional<std::string> expandRemote_;
    long timeout_;

    /** External lock.
     */
    boost::optional<std::string> lock_;

    std::map<Command, std::shared_ptr<UP> > commandParsers_;
};

namespace {

class MergeProgress : public vts::MergeProgress {
public:
    MergeProgress(utility::Filedes &&fd, std::time_t period)
        : fd_(std::move(fd)), period_(period), total_(), count_()
    {}

private:
    virtual void expect_impl(std::size_t total) {
        next_ = std::time(nullptr) + period_;
        total_ = total;
        write(total_);
    }

    virtual void tile_impl() {
        ++count_;
        auto now(std::time(nullptr));
        if (now < next_) { return; }

        next_ = now + period_;
        write(count_);
    }

    void write(std::size_t value) {
        const auto tmp(str(boost::format("%d\n") % value));
        const auto *data(tmp.data());
        auto left(tmp.size());

        while (left) {
            auto written(TEMP_FAILURE_RETRY(::write(fd_, data, left)));
            if (-1 == written) {
                LOG(warn2) << "Error writing to progress fd ("
                           << fd_.get() << "): " << errno;
                break;
            }

            left -= written;
            data += written;
        }
    }

    const utility::Filedes fd_;
    const std::time_t period_;
    std::size_t total_;

    std::time_t next_;
    std::size_t count_;
};

void progressConfiguration(po::options_description &options)
{
    options.add_options()
        ("progress.fd"
         , po::value<int>()->default_value(-1)->required()
         , "File descriptor (number) where merge progress is reported.")
        ("progress.period"
         , po::value<std::time_t>()->default_value(10)->required()
         , "Period between individual progress reports.")
        ;
}

void configureProgress(const po::variables_map &vars
                       , vts::GlueCreationOptions &options)
{
    utility::Filedes fd(vars["progress.fd"].as<int>());
    auto period(vars["progress.period"].as<std::time_t>());
    if (!fd.valid()) {
        LOG(info2) << "Progress fd (" << fd.get() << ") is not valid. "
            "Disabling progress logging.";
        return;
    }

    if (period <= 0) {
        throw po::validation_error
            (po::validation_error::invalid_option_value
             , "progress.period");
    }

    // create fd duplicate and ignore fd
    auto dup(fd.dup());
    fd.release();

    options.progress = std::make_shared<MergeProgress>(std::move(dup), period);
}

void configureGeneratesetModifier(const po::variables_map &vars
                       , vts::GlueCreationOptions &options)
{
    if (!vars.count("debug.tileId")) {
        return;
    }

    vts::TileId tileId(vars["debug.tileId"].as<vts::TileId>());

    options.generateSetManipulator = [=](vts::TileIndex &gs) mutable {
        // prepare tileindex for intersection
        LOG(info2) << "Applying generate set constraint.";

        vts::TileIndex constraint(vts::LodRange(0, gs.maxLod()));

        int cnt(0);
        do {
            constraint.set(tileId, 1);
            tileId = vts::parent(tileId);
            ++cnt;
        } while (tileId.lod);
        LOG(info2) << "Generate set constraint has " << cnt << " valid tiles.";

        gs = gs.intersect(constraint);
    };
}

void getValues(Values &values, const po::variables_map &vars
             , const std::string &option)
{
    if (vars.count(option)) {
        const auto &value(vars[option].as<Values>());
        values.insert(values.end(), value.begin(), value.end());
    }
}

void getValues(vts::Tags &values, const po::variables_map &vars
               , const std::string &option)
{
    if (vars.count(option)) {
        const auto &value(vars[option].as<Values>());
        values.insert(value.begin(), value.end());
    }
}

} // namespace

void VtsStorage::lockConfiguration(po::options_description &config)
{
    config.add_options()
        ("lock", po::value<std::string>()
         , "Externally held lock.");
}

void VtsStorage::lockConfigure(const po::variables_map &vars)
{
    if (vars.count("lock")) {
        lock_ = vars["lock"].as<std::string>();
    }
}

void VtsStorage::configuration(po::options_description &cmdline
                               , po::options_description&
                               , po::positional_options_description &pd)
{
    vr::registryConfiguration(cmdline, vr::defaultPath());

    cmdline.add_options()
        ("path", po::value(&path_)->required()
         , "Path to VTS entity (tileset, storage, storage view) "
         "to work with.")
        ("command", po::value(&command_)
         ->default_value(Command::info)->required()
         , "Command to run.")
        ("noexcept", "Do not catch exceptions, let the program crash.")
        ;

    pd.add("path", 1);

    createParser(cmdline, Command::info
                 , "--command=info: show VTS tileset/storage/storageview info"
                 , [&](UP &p)
    {
        p.options.add_options()
            ("verbose,V", po::value<Verbosity>(&verbose_)->zero_tokens()
             , "Verbose output.")
            ;
    });

    createParser(cmdline, Command::save
                 , "--command=save: save tileset/storage/storageview "
                 "configuration with optional processing"
                 , [&](UP &p)
    {
        p.options.add_options()
            ("output", po::value(&outputPath_)->required()
             , "Dump path.")
            ("expandRemote"
             , utility::implicit_value<std::string>(nullptr, "")
             , "Expand remote entities before saving. Can be set to a base "
             "URL used for non-absolute paths.")
            ("timeout", po::value(&timeout_)->default_value(timeout_)
             , "HTTP timeout, in ms.")
            ;

        p.configure = [&](const po::variables_map &vars) {
            if (vars.count("expandRemote")) {
                expandRemote_ = vars["expandRemote"].as<std::string>();
            }
        };
    });

    createParser(cmdline, Command::create
                 , "--command=create: creates new VTS storage"
                 , [&](UP &p)
    {
        p.options.add_options()
            ("overwrite", "Overwrite existing storage.")
            ("referenceFrame"
             , po::value(&storageProperties_.referenceFrame)->required()
             , "Frame of reference.")
            ;

        p.configure = [&](const po::variables_map &vars) {
            createMode_ = (vars.count("overwrite")
                           ? vts::CreateMode::overwrite
                           : vts::CreateMode::failIfExists);
            referenceFrame_
            = vr::system.referenceFrames(storageProperties_.referenceFrame);
        };
    });

    createParser(cmdline, Command::add
                 , "--command=add: adds new tileset to VTS storage"
                 , [&](UP &p)
    {
        lockConfiguration(p.options);

        p.options.add_options()
            ("tileset", po::value(&tileset_)->required()
             , "Path to source tileset.")
            ("tilesetId", po::value<std::string>()
             , "TilesetId to use in storage, defaults to id "
             "stored in tileset.")

            ("above", po::value<std::string>()
             , "Place new tileset right above given one."
             " Conflicts with --below, --top and --bottom.")
            ("below", po::value<std::string>()
             , "Place new tileset right below given one."
             " Conflicts with --above, --top and --bottom.")
            ("top", "Place new tileset at the top of the stack."
             " Conflicts with --above, --below and --bottom.")
            ("bottom", "Place new tileset at the bottom of the stack."
             " Conflicts with --above, --below and --top.")

            ("lodRange", po::value<vts::LodRange>()
             , "Limits used LOD range from source tileset.")
            ("textureQuality", po::value(&addOptions_.textureQuality)
             ->required()->default_value(addOptions_.textureQuality)
             , "Quality of repacked atlases. 0 means no repacking.")
            ("checkTileindexIdentity"
             , po::value(&addOptions_.checkTileindexIdentity)
             ->required()->default_value(addOptions_.checkTileindexIdentity)
             , "Do not generate a glue if glue candidate contains tilesets "
             "with identical tile indices when set to true.")

            ("bumpVersion", "Add dataset under new version")
            ("dryRun", "Simulate glue creation.")
            ("lazy", "Do not generate any glue. Otherwise work "
             "in legacy mode. In conflict with --full")
            ("full", "Try to generate all glue in lazy mode. Otherwise work "
             "in legacy mode. In conflict with --lazy")
            ("tmp", po::value<fs::path>()
             , "Temporary directory where to work with temporary data.")

            ("addTag", po::value<std::vector<std::string>>()
             , "Set of tags (string identifiers) assigned to tileset. "
             "Glue rules (stored in user-editable file "
             "storage-path/glue.rules) are applied on tags and glues that "
             "result in incompatible combination of tags are not generated.")
            ;

        vts::configuration(p.options, addOptions_);

        progressConfiguration(p.options);

        p.positional.add("tileset", 1);

        p.configure = [&](const po::variables_map &vars) {
            lockConfigure(vars);

            vts::configure(vars, addOptions_);

            if (vars.count("tilesetId")) {
                optTilesetId_ = vars["tilesetId"].as<std::string>();
            }
            if (vars.count("lodRange")) {
                addOptions_.filter.lodRange
                (vars["lodRange"].as<vts::LodRange>());
            }
            if (vars.count("tmp")) {
                addOptions_.tmp = vars["tmp"].as<fs::path>();
            }

            // handle where options
            bool above(vars.count("above"));
            bool below(vars.count("below"));
            bool top(vars.count("top"));
            bool bottom(vars.count("bottom"));
            int sum(above + below + top + bottom);
            if (!sum) {
                throw po::validation_error
                    (po::validation_error::at_least_one_value_required
                     , "above/below/top/bottom", "none");
            }
            if (sum > 1) {
                throw po::validation_error
                    (po::validation_error::multiple_values_not_allowed
                     , "above/below/top/bottom", "above/below/top/bottom");
            }

            if (above) {
                where_.where = vars["above"].as<std::string>();
                where_.direction = vts::Storage::Location::Direction::above;
            } else if (below) {
                where_.where = vars["below"].as<std::string>();
                where_.direction = vts::Storage::Location::Direction::below;
            } else if (top) {
                where_.where.clear();
                where_.direction = vts::Storage::Location::Direction::below;
            } else if (bottom) {
                where_.where.clear();
                where_.direction = vts::Storage::Location::Direction::above;
            }

            addOptions_.bumpVersion = vars.count("bumpVersion");
            addOptions_.dryRun = vars.count("dryRun");

            // handle add mode
            bool lazyMode(vars.count("lazy"));
            bool fullMode(vars.count("full"));
            if (lazyMode && fullMode) {
                throw po::validation_error
                    (po::validation_error::multiple_values_not_allowed
                     , "lazy,full");
            }

            if (lazyMode) {
                addOptions_.mode = vts::Storage::AddOptions::Mode::lazy;
            } else if (fullMode) {
                addOptions_.mode = vts::Storage::AddOptions::Mode::full;
            }

            getValues(addOptions_.tags, vars, "addTag");

            configureProgress(vars, addOptions_);
        };
    });

    createParser(cmdline, Command::remove
                 , "--command=remove: removes tileset from VTS storage"
                 , [&](UP &p)
    {
        lockConfiguration(p.options);

        p.options.add_options()
            ("tileset", po::value(&tilesetIds_)->required()
             , "Id of tileset to remove (can be used more than once).")
            ;

        p.positional.add("tileset", -1);

        p.configure = [&](const po::variables_map &vars) {
            lockConfigure(vars);
        };
    });

    createParser(cmdline, Command::generateGlues
                 , "--command=glue-generate-pending: generates all "
                 "pending glues for given tileset"
                 , [&](UP &p)
    {
        lockConfiguration(p.options);

        p.options.add_options()
            ("tileset", po::value(&tilesetId_)->required()
             , "Id of tileset to generated pending glues for.")

            ("textureQuality", po::value(&addOptions_.textureQuality)
             ->required()->default_value(addOptions_.textureQuality)
             , "Quality of repacked atlases. 0 means no repacking.")

            ("tmp", po::value<fs::path>()
             , "Temporary directory where to work with temporary data.")

            ;

        vts::configuration(p.options, addOptions_);

        progressConfiguration(p.options);

        p.positional.add("tileset", 1);

        p.configure = [&](const po::variables_map &vars) {
            lockConfigure(vars);

            vts::configure(vars, addOptions_);

            if (vars.count("tmp")) {
                addOptions_.tmp = vars["tmp"].as<fs::path>();
            }

            configureProgress(vars, addOptions_);
        };
    });

    createParser(cmdline, Command::generateGlue
                 , "--command=glue-generate: generates given glue if pending"
                 "\n\n  Fails if glue is not pending unless "
                 "explicitly asked to overwrite it (--overwrite)"
                 "\n  or to be benevolent (--onlyIfPending).\n"
                 "\n  Options"
                 , [&](UP &p)
    {
        lockConfiguration(p.options);

        p.options.add_options()
            ("tileset", po::value(&tilesetIds_)->composing()->required()
             , "Tileset participating in glue.")

            ("textureQuality", po::value(&addOptions_.textureQuality)
             ->required()->default_value(addOptions_.textureQuality)
             , "Quality of repacked atlases. 0 means no repacking.")

            ("tmp", po::value<fs::path>()
             , "Temporary directory where to work with temporary data.")

            ("overwrite", "Overwrite existing glue, i.e. regenerate.")
            ("onlyIfPending"
             , "Generate glue only if pending. Ignore any other glue state.")

            ("debug.tileId", po::value<vts::TileId>()
             , "Limits glue to tiles in the path to "
             "given tileId (optional).")
            ;

        vts::configuration(p.options, addOptions_);

        progressConfiguration(p.options);

        p.positional.add("tileset", -1);

        p.configure = [&](const po::variables_map &vars) {
            lockConfigure(vars);

            vts::configure(vars, addOptions_);

            if (vars.count("tmp")) {
                addOptions_.tmp = vars["tmp"].as<fs::path>();
            }

            const bool overwrite(vars.count("overwrite"));
            const bool onlyIfPending(vars.count("onlyIfPending"));

            if (overwrite && onlyIfPending) {
                throw po::validation_error
                    (po::validation_error::multiple_values_not_allowed
                     , "overwrite,onlyIfPending");
            }

            if (overwrite) {
                addOptions_.collisionCheck = false;
            } else if (onlyIfPending) {
                addOptions_.collisionCheck = boost::indeterminate;
            }

            configureProgress(vars, addOptions_);
            configureGeneratesetModifier(vars, addOptions_);
        };
    });

    createParser(cmdline, Command::dumpMetatile
                 , "--command=dump-metatile: dump metatile from tileset"
                 , [&](UP &p)
    {
        p.options.add_options()
            ("tileId", po::value(&tileId_)->required()
             , "ID of any tile inside metatile.")
            ("srs", po::value<std::string>()
             , "Limit output to nodes with given SDS SRS.")
            ("filter", po::value(&metaFlags_)->default_value(0)
             , "Limit output to nodes that match given flags.")
            ;

        p.configure = [&](const po::variables_map &vars) {
            if (vars.count("srs")) {
                optSrs_ = vars["srs"].as<std::string>();
            }
        };

        p.positional.add("tileId", 1);
    });

    createParser(cmdline, Command::dumpMetatileFile
                 , "--command=dump-metatile-file: dump metatile from file"
                 , [&](UP &p)
    {
        p.options.add_options()
            ("referenceFrame", po::value<std::string>()->required()
             , "Reference frame this metatile is for.")
            ("tileId", po::value(&tileId_)->required()
             , "ID of any tile inside metatile.")
            ("srs", po::value<std::string>()
             , "Limit output to nodes with given SDS SRS.")
            ("filter", po::value(&metaFlags_)->default_value(0)
             , "Limit output to nodes that match given flags.")
            ;

        p.configure = [&](const po::variables_map &vars) {
            if (vars.count("srs")) {
                optSrs_ = vars["srs"].as<std::string>();
            }

            referenceFrame_  = vr::system.referenceFrames
            (vars["referenceFrame"].as<std::string>());
        };

        p.positional.add("referenceFrame", 1);
        p.positional.add("tileId", 1);
    });

    createParser(cmdline, Command::metatileVersion
                 , "--command=metatile-version: "
                 "get metatile version from tileset"
                 , [&](UP &p)
    {
        p.options.add_options()
            ("tileId", po::value(&tileId_)->required()
             , "ID of any tile inside metatile.")
            ;

        p.positional.add("tileId", 1);
    });

    createParser(cmdline, Command::mapConfig
                 , "--command=map-config: dump tileset/storage[view] "
                 "map-config"
                 , [&](UP &p)
    {
        p.options.add_options()
            ("proxy", po::value<std::string>()
             , "Proxy name for external URL selection. Storage(view) only.")
            ;

        p.configure = [&](const po::variables_map &vars) {
            if (vars.count("proxy")) {
                mco_.proxy = vars["proxy"].as<std::string>();
            }
        };
    });

    createParser(cmdline, Command::dirs
                 , "--command=dirs: dump tileset/storage[view] "
                 "list of directories"
                 , [&](UP &p)
    {
        p.options.add_options()
            ("proxy", po::value<std::string>()
             , "Proxy name for external URL selection. Storage(view) only.")
            ;

        p.configure = [&](const po::variables_map &vars) {
            if (vars.count("proxy")) {
                mco_.proxy = vars["proxy"].as<std::string>();
            }
        };
    });

    createParser(cmdline, Command::dumpTileIndex
                 , "--command=dump-tileindex: dump tileset's tileindex"
                 , [&](UP &p)
    {
        p.options.add_options()
            ("output", po::value(&outputPath_)->required()
             , "Dump path.")
            ("filter", po::value(&tileFlags_)
             ->default_value(vts::TileIndex::Flag::mesh)
             ->required())
            ;

        p.positional.add("output", 1);
    });

    createParser(cmdline, Command::tileInfo
                 , "--command=tile-info: tile information"
                 , [&](UP &p)
    {
        p.options.add_options()
            ("tileId", po::value(&tileId_)->required()
             , "ID of tile to query.")
            ;

        p.positional.add("tileId", 1);
    });

    createParser(cmdline, Command::tileStatus
                 , "--command=tile-status: tile status"
                 , [&](UP &p)
    {
        p.options.add_options()
            ("tileId", po::value(&tileIds_)->multitoken()
             , "One or more IDs of tile to query. If not tile ID is provided "
             "on the command line IDs are read from stdin")
            ;

        p.positional.add("tileId", -1);
    });

    createParser(cmdline, Command::dumpMesh
                 , "--command=dump-mesh: dump mesh content"
                 , [&](UP &p)
    {
        p.options.add_options()
            ("tileId", po::value(&tileId_)->required()
             , "ID of tile to query.")
            ;
        p.positional.add("tileId", 1);
    });

    createParser(cmdline, Command::dumpMeshMask
                 , "--command=dump-mesh-mask: dump mesh mask as an image"
                 , [&](UP &p)
    {
        p.options.add_options()
            ("tileId", po::value(&tileId_)->required()
             , "ID of tile to query.")
            ("output", po::value(&outputPath_)->required()
             , "Path of output image.")
            ("generate", utility::implicit_value(&generate_, true)
             ->default_value(false))
            ;

        p.positional.add("output", 1);
    });

    createParser(cmdline, Command::exportMesh
                 , "--command=export-mesh: export mesh contents as a "
                 "directory with OBJ files and textures"
                 , [&](UP &p)
    {
        p.options.add_options()
            ("tileId", po::value(&tileId_)->required()
             , "ID of tile to query.")
            ("output", po::value(&outputPath_)
             , "Output directory; defaults to tileId.")
            ("format", po::value(&meshFormat_)
             , utility::concat
             ("Mesh format, one of "
              , enumerationString(meshFormat_), ".").c_str())
            ("margin", po::value(&addOptions_.safetyMargin)
             , "Margin arount tiles extents in pixels of coverage.")
            ;
        p.positional.add("tileId", 1);
        p.positional.add("output", 1);

        p.configure = [&](const po::variables_map &vars) {
            if (!vars.count("output")) {
                outputPath_ = boost::lexical_cast<std::string>(tileId_);
            }
        };
    });

    createParser(cmdline, Command::tileMultifileLayout
                 , "--command=tile-multifile-layout: show multifile layout "
                 "(reads multifile table)"
                 , [&](UP &p)
    {
        p.options.add_options()
            ("tileId", po::value(&tileId_)->required()
             , "ID of tile to query.")
            ("filter", po::value(&tileFlags_)
             ->default_value(vts::TileIndex::Flag::mesh)
             ->required()
             , "File type filter (mesh, altas, navtile)")
            ;

        p.positional.add("tileId", 1);
        p.positional.add("filter", 1);
    });

    createParser(cmdline, Command::tileIndexInfo
                 , "--command=tileindex-info: tile-index query"
                 , [&](UP &p)
    {
        p.options.add_options()
            ("tileId", po::value(&tileIds_)
             , "ID's of tiles to query.")
            ;
        p.positional.add("tileId", -1);
    });

    createParser(cmdline, Command::tileIndexRanges
                 , "--command=tileindex-ranges: tile-index ranges"
                 , [&](UP &p)
    {
        p.options.add_options()
            ("verbose,V", po::value<Verbosity>(&verbose_)->zero_tokens()
                , "Verbose output.")
            ("filter", po::value(&tileFlags_)
                ->default_value(vts::TileIndex::Flag::mesh)
                ->required())
            ;
    });

    createParser(cmdline, Command::convertTileIndex
                 , "--command=convert-tileindex: tile-index conversion to "
                 "latest version"
                 , [&](UP &p)
    {
        p.options.add_options()
            ("output", po::value(&outputPath_)->required()
             , "Path of output tileindex.")
            ;
        p.positional.add("output", -1);
    });

    createParser(cmdline, Command::concat
                 , "--command=concat: concatenate tilesets into one set"
                 , [&](UP &p)
    {
        p.options.add_options()
            ("overwrite", "Overwrite existing output tileset.")
            ("tileset", po::value(&tilesets_)
             , "Tileset to paste into output tileset.")
            ("lodRange", po::value<vts::LodRange>()
             , "Limits used LOD range from source tileset.")
            ;
        p.positional.add("tileset", -1);

        p.configure = [&](const po::variables_map &vars) {
            createMode_ = (vars.count("overwrite")
                           ? vts::CreateMode::overwrite
                           : vts::CreateMode::failIfExists);

            if (vars.count("lodRange")) {
                optLodRange_ = vars["lodRange"].as<vts::LodRange>();
            }
        };
    });

    createParser(cmdline, Command::aggregate
                 , "--command=aggregate: aggregate more tilesets info "
                 "one virtual tileset"
                 , [&](UP &p)
    {
        p.options.add_options()
            ("output", po::value(&outputPath_)->required()
             , "Path to aggregated tileset.")
            ("tileset", po::value(&tilesetIds_)
             , "Id of tileset to aggreage (mandatory "
             "if working with storage).")
            ("overwrite", "Overwrite existing output tileset.")
            ("tilesetId", po::value<std::string>()
             , "TilesetId of created tileset, defaults to last part of "
             "output path.")
            ("staticMetaLodRange", po::value<MetaLodRange>()
             , "LOD range where metatiles are pre-generated. Use \"full\" "
             "to force whole metatile tree generation.")
            ;
        p.positional.add("tileset", -1);

        p.configure = [&](const po::variables_map &vars) {
            if (vars.count("tilesetId")) {
                optTilesetId_ = vars["tilesetId"].as<std::string>();
            }

            createMode_ = (vars.count("overwrite")
                           ? vts::CreateMode::overwrite
                           : vts::CreateMode::failIfExists);

            if (vars.count("staticMetaLodRange")) {
                optLodRange_ = vars["staticMetaLodRange"].as<MetaLodRange>()
                    .lodRange;
            }
        };
    });

    createParser(cmdline, Command::remote
                 , "--command=remote: create remote (HTTP) tileset adapter"
                 , [&](UP &p)
    {
        p.options.add_options()
            ("url", po::value(&remoteUrl_), "Remote (HTTP) tileset URL.")
            ("overwrite", "Overwrite existing output tileset.")
            ("tilesetId", po::value<std::string>()
             , "TilesetId of created tileset, defaults to ID of remote set.")
            ;

        p.configure = [&](const po::variables_map &vars) {
            if (vars.count("tilesetId")) {
                optTilesetId_ = vars["tilesetId"].as<std::string>();
            }

            createMode_ = (vars.count("overwrite")
                           ? vts::CreateMode::overwrite
                           : vts::CreateMode::failIfExists);
        };
    });

    createParser(cmdline, Command::local
                 , "--command=local: create local (filesystem) tileset adapter"
                 , [&](UP &p)
    {
        p.options.add_options()
            ("source", po::value(&localPath_), "Local tileset path.")
            ("overwrite", "Overwrite existing output tileset.")
            ("tilesetId", po::value<std::string>()
             , "TilesetId of created tileset, defaults to ID of local set.")
            ;

        p.configure = [&](const po::variables_map &vars) {
            if (vars.count("tilesetId")) {
                optTilesetId_ = vars["tilesetId"].as<std::string>();
            }

            createMode_ = (vars.count("overwrite")
                           ? vts::CreateMode::overwrite
                           : vts::CreateMode::failIfExists);
        };
    });

    createParser(cmdline, Command::clone
                 , "--command=clone: clone existing tileset"
                 , [&](UP &p)
    {
        p.options.add_options()
            ("tileset", po::value(&tileset_)->required()
             , "Path to output tileset.")
            ("overwrite", "Overwrite existing output tileset.")
            ("tilesetId", po::value<std::string>()
             , "TilesetId of output tileset. Defaults to filename of "
             "tileset path ")
            ("lodRange", po::value<vts::LodRange>()
             , "Limits output to given LOD range from source tileset.")
            ("forceCredits", po::value<std::string>()
             , "Comma-separated list of string/numeric credit id to override "
             "existing credits. If not specified, credits are not touched.")
            ("encode", po::value(&encodeFlags_)->default_value(0)
             ,"Comma-separated list of clone options: mesh, inpaint, meta.")
            ("textureQuality", po::value(&textureQuality_)
             ->required()->default_value(textureQuality_)
             , "Texture quality for inpaint.")
            ("sameType", utility::implicit_value(&sameType_, true)
             ->default_value(false)
             , "Clones tileset as-is if true. Otherwise clones all data "
             "using plain driver.")
            ;

        p.configure = [&](const po::variables_map &vars) {
            if (vars.count("tilesetId")) {
                optTilesetId_ = vars["tilesetId"].as<std::string>();
            }

            createMode_ = (vars.count("overwrite")
                           ? vts::CreateMode::overwrite
                           : vts::CreateMode::failIfExists);

            if (vars.count("lodRange")) {
                optLodRange_ = vars["lodRange"].as<vts::LodRange>();
            }

            if (vars.count("forceCredits")) {
                std::vector<std::string> parts;
                for (const auto &value
                    : ba::split( parts, vars["forceCredits"].as<std::string>()
                               , ba::is_any_of(",")))
                {
                    vr::Credit credit;
                    int numericCredit(-1);
                    try {
                        numericCredit = boost::lexical_cast<int>(value);
                    } catch (const boost::bad_lexical_cast&) {
                        credit = vr::system.credits(value);
                    }

                    try {
                        credit = vr::system.credits(numericCredit);
                    } catch (const vs::KeyError&) {
                        LOG(warn2) << "Using numeric credit id: "
                                   << numericCredit << " unknown in registry";
                        forceCredits_.insert(numericCredit);
                        continue;
                    }

                    forceCredits_.insert(credit.numericId);
                }
            }
        };

        p.positional.add("tileset", 1);
    });

    createParser(cmdline, Command::relocate
                 , "--command=relocate: update paths to external resources "
                 "in a dataset"
                 , [&](UP &p)
    {
        p.options.add_options()
            ("dryRun", "Simulate relocate.")
            ("rule", po::value(&relocateOptions_.rules)
             , "Rule in form prefix=replacement. Can be use multiple times. "
             "First matching rule is applied. Can be omitted to display "
             "dependency tree.")
            ;

        p.configure = [&](const po::variables_map &vars) {
            relocateOptions_.dryRun = vars.count("dryRun");

            if (!relocateOptions_.dryRun && relocateOptions_.rules.empty()) {
                throw po::required_option("rules");
            }
        };
    });

    createParser(cmdline, Command::reencode
                 , "--command=reencode: recursively reencode tilesets"
                 , [&](UP &p)
    {
        p.options.add_options()
            ("dryRun", "Simulate reencode.")
            ("tag", po::value(&reencodeOptions_.tag)->required()
             , "Reencode tag.")
            ("encode", po::value(&encodeFlags_)->default_value(0)
             ,"Comma-separated list of clone options: mesh, inpaint, meta.")
            ;

        p.configure = [&](const po::variables_map &vars) {
            reencodeOptions_.dryRun = vars.count("dryRun");
        };
    });

    createParser(cmdline, Command::reencodeCleanup
                 , "--command=reencodeCleanup: cleanup after previous reencode"
                 , [&](UP &p)
    {
        p.options.add_options()
            ("dryRun", "Simulate reencode cleanup.")
            ("tag", po::value(&reencodeOptions_.tag)->required()
             , "Reencode tag.")
            ;

        p.configure = [&](const po::variables_map &vars) {
            reencodeOptions_.dryRun = vars.count("dryRun");
        };
    });

    createParser(cmdline, Command::tilePick
                 , "--command=tile-pick: create new tiles by picking "
                 "enumerated tiles from source"
                 , [&](UP &p)
    {
        p.options.add_options()
            ("tileset", po::value(&tileset_)->required()
             , "Path to created tileset.")
            ("overwrite", "Overwrite existing output tileset.")
            ("tileId", po::value(&tileIds_)
             , "ID of tile to pick (can be specified multiple times).")
            ;

        p.configure = [&](const po::variables_map &vars) {
            createMode_ = (vars.count("overwrite")
                           ? vts::CreateMode::overwrite
                           : vts::CreateMode::failIfExists);
        };
    });

    createParser(cmdline, Command::tags
                 , "--command=tags: operate on tileset's tags inside storage"
                 , [&](UP &p)
    {
        p.options.add_options()
            ("tileset", po::value<std::string>()
             , "Id of tileset which tags to edit or display. Optional "
             "when dislaying tag info (all tilesets are shown).")
            ("addTag", po::value<std::vector<std::string>>()
             , "Tags to add. All positional arguments are treated as tags "
             "to be added")
            ("removeTag", po::value<std::vector<std::string>>()
             , "Tags to remove.")
            ;

        p.positional.add("addTag", -1);

        p.configure = [&](const po::variables_map &vars) {
            getValues(addValues_, vars, "addTag");
            getValues(removeValues_, vars, "removeTag");

            if (!addValues_.empty() || !removeValues_.empty()) {
                if (!vars.count("tileset")) {
                    throw po::required_option("tileset");
                }

                tilesetId_ = vars["tileset"].as<std::string>();
            } else if (vars.count("tileset")) {
                optTilesetId_ = vars["tileset"].as<std::string>();
            }
        };
    });

    createParser(cmdline, Command::proxies
                 , "--command=proxies: operate on tileset's proxies "
                 "inside storage"
                 , [&](UP &p)
    {
        p.options.add_options()
            ("tileset", po::value<std::string>()
             , "Id of tileset which proxies to edit or display. Optional "
             "when dislaying tag info (all tilesets are shown).")
            ("addProxy", po::value<std::vector<std::string>>()
             , "Proxies to add in format name=URL. All positional arguments "
             "are treated as proxies to be added")
            ("removeProxy", po::value<std::vector<std::string>>()
             , "Proxy names to remove.")
            ;

        p.positional.add("addProxy", -1);

        p.configure = [&](const po::variables_map &vars) {
            getValues(addValues_, vars, "addProxy");
            getValues(removeValues_, vars, "removeProxy");

            // sanity check
            for (const auto &value : addValues_) {
                if (!value.find('=')) {
                    throw po::validation_error
                        (po::validation_error::invalid_option_value
                         , "addProxy");
                }
            }


            if (!addValues_.empty() || !removeValues_.empty()) {
                if (!vars.count("tileset")) {
                    throw po::required_option("tileset");
                }

                tilesetId_ = vars["tileset"].as<std::string>();
            } else if (vars.count("tileset")) {
                optTilesetId_ = vars["tileset"].as<std::string>();
            }
        };
    });

    createParser(cmdline, Command::file
                 , "--command=file: fetch file from dataset"
                 , [&](UP &p)
    {
        p.options.add_options()
            ;
    });

    createParser(cmdline, Command::mergeConfSyntax
                 , "--command=merge-conf-syntax: "
                 "show syntax of merge configuration"
                 , [&](UP &p)
    {
        p.options.add_options()
            ;
    });

    createParser(cmdline, Command::dumpNavtile
                 , "--command=dump-navtile: navtile"
                 , [&](UP &p)
    {
        p.options.add_options()
            ("tileId", po::value(&tileId_)->required()
             , "ID of tile to query.")
            ("output", po::value(&outputPath_)->required()
             , "Path of output image.")
            ;

        p.positional.add("output", 1);
    });

    createParser(cmdline, Command::dumpNavtileMask
                 , "--command=dump-navtile-mask: navtile mask as an image"
                 , [&](UP &p)
    {
        p.options.add_options()
            ("tileId", po::value(&tileId_)->required()
             , "ID of tile to query.")
            ("output", po::value(&outputPath_)->required()
             , "Path of output image.")
            ;

        p.positional.add("output", 1);
    });

    createParser(cmdline, Command::navtile2dem
                 , "--command=navtile2dem: write navtile as GDAL DEM"
                 , [&](UP &p)
    {
        p.options.add_options()
            ("tileId", po::value(&tileId_)->required()
             , "ID of tile to query.")
            ("output", po::value(&outputPath_)->required()
             , "Path of output GDAL dataset (geo tiff).")
            ;

        p.positional.add("output", 1);
    });

    createParser(cmdline, Command::showLockerApi
                 , "--command=show-locker-api: "
                 "show API of external locking program"
                 , [&](UP &p)
    {
        p.options.add_options()
            ;
    });

    createParser(cmdline, Command::deriveMetaIndex
                 , "--command=derive-metaindex: "
                 "generate metaindex from tileindex"
                 , [&](UP &p)
    {
        p.options.add_options()
            ("output", po::value(&outputPath_)->required()
             , "Path to generated metaindex file.")
            ;
    });

    createParser(cmdline, Command::virtualSurfaceCreate
                 , "--command=vs-create: creates new virtual "
                 "surface in VTS storage"
                 , [&](UP &p)
    {
        lockConfiguration(p.options);

        p.options.add_options()
            ("tileset", po::value(&tilesetIds_)
             , "Id of tileset to aggreage (mandatory "
             "if working with storage).")
            ("overwrite", "Overwrite existing output tileset.")
            ("staticMetaLodRange", po::value<MetaLodRange>()
             , "LOD range where metatiles are pre-generated. Use \"full\" "
             "to force whole metatile tree generation.")
            ("tilesetId", po::value<std::string>()
             , "ID of virtual surface's tileset: used both for an internal ID "
             " and as a directory name. Default is underscore-joined list of "
             "constituent surfaces.")
            ;
        p.positional.add("tileset", -1);

        p.configure = [&](const po::variables_map &vars) {
            lockConfigure(vars);

            createMode_ = (vars.count("overwrite")
                           ? vts::CreateMode::overwrite
                           : vts::CreateMode::failIfExists);

            if (vars.count("staticMetaLodRange")) {
                optLodRange_ = vars["staticMetaLodRange"].as<MetaLodRange>()
                    .lodRange;
            }

            if (vars.count("tilesetId")) {
                optTilesetId_ = vars["tilesetId"].as<std::string>();
            }
        };
    });

    createParser(cmdline, Command::virtualSurfaceRemove
                 , "--command=vs-remove: removes a virtual "
                 "surface from a VTS storage"
                 , [&](UP &p)
    {
        lockConfiguration(p.options);

        p.options.add_options()
            ("tileset", po::value(&tilesetIds_)
             , "Id of tileset to de-aggregate (mandatory "
             "if working with storage).")
            ;
        p.positional.add("tileset", -1);

        p.configure = [&](const po::variables_map &vars) {
            lockConfigure(vars);
        };
    });

    createParser(cmdline, Command::virtualSurfaceInfo
                 , "--command=vs-info: show storage's virtual "
                 "surface info"
                 , [&](UP &p)
    {
        p.options.add_options()
            ("tileset", po::value(&tilesetIds_)->multitoken()
             , "Id of tileset for info (mandatory "
             "if working with storage).")
            ;
        p.positional.add("tileset", -1);
    });

    createParser(cmdline, Command::checkMetatileTree
                 , "--command=check-metatile-tree: walk the metatile tree"
                 , [&](UP &p)
    {
        p.options.add_options()
            ("tileId", po::value(&tileId_)->required()
             , "ID of any tile inside root metatile.")
            ;

        p.positional.add("tileId", 1);
    });

    createParser(cmdline, Command::queryNavtile
                 , "--command=query-navtile: query navtiles at given position "
                 "and LOD"
                 , [&](UP &p)
    {
        p.options.add_options()
            ("lod", po::value<vts::Lod>()
             , "LOD at which to query.")
            ("point", po::value(&queryPoint_)->required()
             , "Point (in navigation SRS) to query.")
            ;
        p.positional.add("point", 1);

        p.configure = [&](const po::variables_map &vars) {
            if (vars.count("lod")) {
                queryLod_ = vars["lod"].as<vts::Lod>();
            }
        };
    });

    createParser(cmdline, Command::listPendingGlues
                 , "--command=list-pending-glues: "
                 "list pending glues missing to display storage/storageview."
                 , [&](UP&)
    {});

    createParser(cmdline, Command::locker2Stresser
                 , "--command=locker2-stresser: "
                 "stress locker implementation"
                 , [&](UP &p)
    {
        (void) p;
    });

    createParser(cmdline, Command::decodeTsMap
                 , "--command=decode-tsmap: "
                 "decodes tileset.map file"
                 , [&](UP &p)
    {
        (void) p;
    });

    createParser(cmdline, Command::listReferenceFrames
                 , "--command=list-reference-frames: "
                 "list available reference frames"
                 , [&](UP &p)
    {
        (void) p;
    });
}

po::ext_parser VtsStorage::extraParser()
{
    return [&](const std::string &s) -> std::pair<std::string, std::string>
    {
        if ((s.size() < 3) || (s[0] != '-') || (s[1] != '-')) {
            return {};
        }

        // translate standalone --COMMAND_NAME into --command=COMMAND_NAME
        for (const auto &p : commandParsers_) {
            const auto &name(boost::lexical_cast<std::string>(p.first));
            if (!s.compare(2, std::string::npos, name)) {
                return { "command", name };
            }
        }
        return {};
    };
}

UP::optional
VtsStorage::configure(const po::variables_map &vars
                 , const std::vector<std::string>&)
{
    vr::registryConfigure(vars);

    if (!vars.count("command")) { return {}; }
    return getParser(vars["command"].as<Command>());
}

UP::optional VtsStorage::getParser(Command command)
    const
{
    auto fcommandParsers(commandParsers_.find(command));
    if ((fcommandParsers != commandParsers_.end())
         && fcommandParsers->second)
    {
        return *fcommandParsers->second;
    }

    return {};
}

void VtsStorage::configure(const po::variables_map &vars)
{
    noexcept_ = vars.count("noexcept");
}

std::vector<std::string> VtsStorage::listHelps() const
{
    std::vector<std::string> out;

    for (const auto &p : commandParsers_) {
        out.push_back(boost::lexical_cast<std::string>(p.first));
    }

    return out;
}

bool VtsStorage::help(std::ostream &out, const std::string &what) const
{
    if (what.empty()) {
        // program help
        out << ("Tile archive manipulator\n"
                );

        return true;
    }

    try {
        if (auto p = getParser(boost::lexical_cast<Command>(what))) {
            out << p->options;
        }
        return true;
    } catch (const boost::bad_lexical_cast&) {}

    return false;
}

int VtsStorage::runCommand()
{

#define VTS_COMMAND_LIST_distribute(r, NOTHING, value)                 \
    case Command::BOOST_PP_SEQ_ELEM(0, value):                         \
        return BOOST_PP_SEQ_ELEM(0, value)();

    switch (command_) {
        BOOST_PP_SEQ_FOR_EACH(VTS_COMMAND_LIST_distribute
                              , NOTHING, VTS_COMMAND_LIST)
    }
    std::cerr << "vts: no operation requested" << '\n';
    return EXIT_FAILURE;

#undef VTS_COMMAND_LIST_distribute
}

int VtsStorage::run()
{
    if (noexcept_) {
        return runCommand();
    }

    try {
        return runCommand();
    } catch (const std::exception &e) {
        std::cerr << "vts: " << e.what() << '\n';
        return EXIT_FAILURE;
    }
}

int tilesetInfo(const fs::path &path, int verbose);
int storageInfo(const fs::path &path, int verbose);
int storageViewInfo(const fs::path &path, int verbose);

void tiInfo(const vts::TileIndex &ti, const std::string &prefix = "")
{
    typedef vts::TileIndex::Flag TiFlag;
    typedef std::pair<TiFlag::value_type, TiFlag::value_type> Flags;
    for (auto flag : { Flags(TiFlag::mesh, TiFlag::mesh)
                , Flags(TiFlag::atlas, TiFlag::atlas)
                , Flags(TiFlag::navtile, TiFlag::navtile)
                , Flags(TiFlag::alien | TiFlag::mesh
                        , TiFlag::alien | TiFlag::mesh)
                , Flags(TiFlag::influencedMask, TiFlag::influenced)
                })
    {
        auto stat(ti.statMask(flag.first, flag.second));
        std::cout
            << prefix << "    " << vts::TileFlags(flag.second)
            << ":" << '\n'
            << prefix << "        lodRange: " << stat.lodRange << '\n'
            << prefix << "        count = " << stat.count << '\n'
            ;

        // special handling for mesh: make statistics for watertight
        if (flag.first == vts::TileIndex::Flag::mesh) {
            auto wstat(ti.statMask(vts::TileIndex::Flag::watertight));
            std::cout
                << prefix << "        watertight = " << wstat.count
                << '\n';
        }
    }
}

void miInfo(const vts::TileIndex &ti, const vr::ReferenceFrame &rf
            , const std::string &prefix = "")
{
    const auto metaCount(ti.shrinkedCount(rf.metaBinaryOrder));
    // get tile range and make from zero if non-empty
    auto lr(ti.lodRange());
    if (!lr.empty()) { update(lr, vts::Lod(0)); }

    std::cout
            << prefix << "    meta:\n"
            << prefix << "        lodRange: " << lr << '\n'
            << prefix << "        count = " << metaCount << '\n'
            ;
}

int tilesetInfo(const std::string &prefix, const fs::path &path
                , int verbose)
{
    auto ts(vts::openTileSet(path));
    auto prop(ts.getProperties());
    if (!verbose) {
        std::cout << prefix << prop.id << " [" << ts.typeInfo()
                  << "]" << '\n';

    } else {
        std::cout << prefix << "Id: " << prop.id << '\n';
        std::cout << prefix << "Type: " << ts.typeInfo() << '\n';
        std::cout << prefix << "Reference frame: " << ts.referenceFrame().id
                  << '\n';

        std::cout << prefix << "Tile type info:" << '\n';
        miInfo(ts.tileIndex(), ts.referenceFrame(), prefix);
        tiInfo(ts.tileIndex(), prefix);

        if (verbose > 1) {
            std::cout << prefix << "texelSize: " << std::fixed
                      << ts.texelSize() << '\n';
        }
    }

    return EXIT_SUCCESS;
}

int storageInfo(const std::string &prefix, const fs::path &path, int verbose)
{
    auto s(vts::openStorage(path));
    std::cout << prefix << "Tile sets:" << '\n';
    for (const auto &tileset : s.storedTilesets()) {
        auto subprefix(prefix + "    ");
        tilesetInfo(subprefix, s.path(tileset.tilesetId), verbose);
        if (verbose) {
            if (!tileset.tags.empty()) {
                std::cout << subprefix << "Tags: "
                          << utility::join(tileset.tags, ", ") << '\n';
            }

            if (!tileset.proxy2ExternalUrl.empty()) {
                std::cout << subprefix << "External URL:\n";
                for (const auto &item : tileset.proxy2ExternalUrl) {
                    std::cout << subprefix << "    " << item.first
                              << " -> " << item.second << '\n';
                }
            }

            std::cout << '\n';
        }
    }

    std::cout << prefix << "Glues:" << '\n';
    for (const auto &gitem : s.glues()) {
        if (!verbose) {
            std::cout << prefix << "    " << utility::join(gitem.first, ", ")
                      << "\n";
        } else {
            std::cout
                << prefix << "    Glue-Id: "
                << utility::join(gitem.first, ", ") << '\n';

            tilesetInfo(prefix + "    ",  s.path(gitem.second), verbose);
            std::cout << '\n';
        }
    }
    for (const auto &gid : s.pendingGlues()) {
        std::cout << prefix << "    [pending] " << utility::join(gid, ", ")
                  << "\n";
    }

    const auto vs(s.virtualSurfaces());
    if (!vs.empty()) {
        std::cout << prefix << "Virtual surfaces:" << '\n';
        for (const auto &vsitem : s.virtualSurfaces()) {
            if (!verbose) {
                std::cout
                    << prefix << "    " << utility::join(vsitem.first, ", ")
                    << '\n';
            } else {
                std::cout << prefix << "    VirtualSurface-Id: "
                          << utility::join(vsitem.first, ", ")
                          << '\n';

                tilesetInfo(prefix + "    ",  s.path(vsitem.second), verbose);
                std::cout << '\n';
            }
        }
    }

    return EXIT_SUCCESS;
}

int storageViewInfo(const std::string &prefix, const fs::path &path
                    , int verbose)
{
    auto sv(vts::openStorageView(path));
    std::cout << prefix << "View into storage:" << '\n'
              << prefix << "    " << sv.storagePath() << '\n';
    std::cout << prefix << "Tile sets:" << '\n';
    for (const auto &tid : sv.tilesets()) {
        std::cout << prefix << "    " << tid << '\n';
    }
    return EXIT_SUCCESS;
    (void) verbose;
}

int VtsStorage::info()
{
    switch (vts::datasetType(path_)) {
    case vts::DatasetType::TileSet:
        return tilesetInfo(std::string(), path_, verbose_);

    case vts::DatasetType::Storage:
        return storageInfo(std::string(), path_, verbose_);

    case vts::DatasetType::StorageView:
        return storageViewInfo(std::string(), path_, verbose_);

    case vts::DatasetType::TileIndex:
        std::cerr << "Tile index is not supported." << '\n';
        return EXIT_FAILURE;

    default: break;
    }

    std::cerr << "Unrecognized content " << path_ << "." << '\n';
    return EXIT_FAILURE;
}

int saveStorageView(const fs::path &path, const fs::path &outputPath
                    , const boost::optional<std::string> &expandRemote
                    , long timeout)
{
    const auto filter([&](vts::ExtraStorageProperties &extra) -> void
    {
        if (!expandRemote) { return; }

        tools::UrlFetcher fetcher(timeout);
        vr::UrlExpander e(*expandRemote, [&fetcher](const std::string &url) {
                return fetcher.fetch(url);
            });

        extra.boundLayers = e.expand(extra.boundLayers);
        extra.freeLayers = e.expand(extra.freeLayers);
    });

    vts::openStorageView(path).saveConfig(outputPath, filter, true);

    return EXIT_SUCCESS;
}

int VtsStorage::save()
{
    switch (vts::datasetType(path_)) {
    case vts::DatasetType::TileSet:
        std::cerr << "Save of tileset's configuration not supproted yet.";
        return EXIT_FAILURE;

    case vts::DatasetType::Storage:
        std::cerr << "Save of storage's configuration not supproted yet.";
        return EXIT_FAILURE;

    case vts::DatasetType::TileIndex:
        std::cerr << "There is no config to save in tileindex.";
        return EXIT_FAILURE;

    case vts::DatasetType::StorageView:
        return saveStorageView(path_, outputPath_, expandRemote_, timeout_);

    default: break;
    }

    std::cerr << "Unrecognized content " << path_ << "." << '\n';
    return EXIT_FAILURE;
}

int VtsStorage::create()
{
    auto storage(vts::Storage(path_, storageProperties_, createMode_));

    return EXIT_SUCCESS;
}

namespace {

bool isLocal(const fs::path &path)
{
    return ba::starts_with(path.string(), "local:");
}

fs::path localPart(const fs::path &path)
{
    if (isLocal(path)) { return path.string().substr(6); }
    return path;
}

bool isRemote(const fs::path &path)
{
    return (ba::istarts_with(path.string(), "http://")
            || ba::istarts_with(path.string(), "https://")
            || ba::istarts_with(path.string(), "//"));
}

} // namespace

int VtsStorage::add()
{
    const auto tmpPath(path_ / "tmp/tileset-to-add");

    // lock if external locking program is available
    Lock lock(path_, lock_);
    auto storage(vts::Storage(path_, vts::OpenMode::readWrite, lock));

    if (isLocal(tileset_)) {
        // local: URL, create temporary tileset
        auto local(localPart(tileset_));

        LOG(info3)
            << "Requested to add local tileset pointing to " << local
            << ". Generating temporary local tileset.";

        vts::CloneOptions createOptions;
        createOptions.tilesetId(optTilesetId_);
        createOptions.mode(vts::CreateMode::overwrite);
        vts::createLocalTileSet
            (tmpPath, local, createOptions);
        tileset_ = tmpPath;
    } else if (isRemote(tileset_)) {
        // http:// or https:// URL, create remote tileset

        LOG(info3)
            << "Requested to add remote tileset pointing to " << tileset_
            << ". Generating temporary remote tileset.";

        vts::CloneOptions createOptions;
        createOptions.tilesetId(optTilesetId_);
        createOptions.mode(vts::CreateMode::overwrite);

        vts::createRemoteTileSet(tmpPath, tileset_.string(), createOptions);
        tileset_ = tmpPath;
    }

    storage.add(tileset_, where_
                , optTilesetId_ ? *optTilesetId_ : std::string()
                , addOptions_);

    LOG(info4) << "All done.";
    return EXIT_SUCCESS;
}

int VtsStorage::remove()
{
    // lock if external locking program is available
    Lock lock(path_, lock_);
    auto storage(vts::Storage(path_, vts::OpenMode::readWrite, lock));

    storage.remove(tilesetIds_);
    return EXIT_SUCCESS;
}

int VtsStorage::generateGlues()
{
    // lock if external locking program is available
    Lock lock(path_, lock_);
    auto storage(vts::Storage(path_, vts::OpenMode::readWrite, lock));

    auto ao(addOptions_);
    ao.mode = vts::Storage::AddOptions::Mode::full;
    storage.generateGlues(tilesetId_, ao);
    return EXIT_SUCCESS;
}

int VtsStorage::generateGlue()
{
    // lock if external locking program is available
    Lock lock(path_, lock_);
    auto storage(vts::Storage(path_, vts::OpenMode::readWrite, lock));

    auto ao(addOptions_);
    ao.mode = vts::Storage::AddOptions::Mode::full;
    storage.generateGlue(tilesetIds_, ao);
    return EXIT_SUCCESS;
}

int VtsStorage::tags()
{
    auto print([&](const vts::Storage &storage
                   , const boost::optional<vts::TilesetId> &filterOut)
    {
        for (const auto &tileset : storage.storedTilesets()) {
            if (filterOut && (tileset.tilesetId != filterOut)) {
                continue;
            }
            std::cout << tileset.tilesetId << ": "
                  << utility::join(tileset.tags, ", ") << '\n';
        }
    });

    if (!addValues_.empty() || !removeValues_.empty()) {
        // lock if external locking program is available
        Lock lock(path_, lock_);
        auto storage(vts::Storage(path_, vts::OpenMode::readWrite, lock));

        storage.updateTags(tilesetId_, tagsFromValues(addValues_)
                           , tagsFromValues(removeValues_));

        print(storage, tilesetId_);
    } else {
        auto storage(vts::Storage(path_, vts::OpenMode::readWrite));
        print(storage, optTilesetId_);
    }
    return EXIT_SUCCESS;
}

int VtsStorage::proxies()
{
    auto applyFilter([](const boost::optional<vts::TilesetId> &filterOut
                        , const vts::TilesetId &tilesetId) -> bool
    {
        return !(filterOut && (tilesetId != *filterOut));
    });

    auto printMapping([](const vts::TilesetId &tilesetId
                         , const vts::Proxy2ExternalUrl &mapping) -> void
    {
        std::cout << tilesetId << "\n";
        for (const auto &item : mapping) {
            std::cout << "    " << item.first
                      << " -> " << item.second << '\n';
        }
    });

    auto print([&](const vts::Storage &storage
                   , const boost::optional<vts::TilesetId> &filterOut)
    {
        for (const auto &tileset : storage.storedTilesets()) {
            if (!applyFilter(filterOut, tileset.tilesetId)) { continue; }
            printMapping(tileset.tilesetId, tileset.proxy2ExternalUrl);
        }

        if (applyFilter(filterOut, "@glues")) {
            printMapping("@glues", storage.gluesExternalUrl());
        }

        if (applyFilter(filterOut, "@vs")) {
            printMapping("@vs", storage.vsExternalUrl());
        }
    });

    if (!addValues_.empty() || !removeValues_.empty()) {
        // lock if external locking program is available
        Lock lock(path_, lock_);
        auto storage(vts::Storage(path_, vts::OpenMode::readWrite, lock));

        storage.updateExternalUrl(tilesetId_, proxiesFromValues(addValues_)
                                  , removeValues_);

        print(storage, tilesetId_);
    } else  {
        auto storage(vts::Storage(path_, vts::OpenMode::readWrite));
        print(storage, optTilesetId_);
    }

    return EXIT_SUCCESS;
}

void showCredits(std::ostream &out, const vts::MetaNode &node
                 , const std::string &prefix)
{
    out << prefix << "credits: ";
    if (node.credits().empty()) {
        out << "none" << '\n';
    } else {
        out << node.credits().size() << '\n';
    }

    for (const auto &credit : node.credits()) {
        out << prefix << "    " << credit;
        if (const auto *info = vr::system.credits(credit, std::nothrow)) {
            out << ": " << info->id;
        } else {
            out << ": <unknown>";
        }

        out << '\n';
    }
    return;
}

int VtsStorage::dumpMetatile()
{
    std::cout << std::fixed;

    auto ts(vts::openTileSet(path_));
    auto rf(ts.referenceFrame());

    auto meta(ts.getMetaTile(tileId_));

    std::cout << "Metatile ID: " << meta.origin() << '\n';

    std::cout << "Parent metatile ID: "
              << ts.metaId(vts::parent(meta.origin())) << '\n';

    {
        auto e(meta.validExtents());
        std::cout << "Covered global tile extents: " << e << '\n';
        if (!math::valid(e)) {
            std::cout << "No valid tile in this metatile." << '\n';
            return EXIT_SUCCESS;
        }
    }

    std::set<vts::TileId> metaChildIds;

    meta.for_each([&](const vts::TileId &tid, const vts::MetaNode &node)
    {
        auto tiFlags(ts.tileIndex().get(tid));
        if (!tiFlags && !node.flags()) { return; }

        // filter out nodes with content not needed
        if (metaFlags_.value && !(metaFlags_.value & node.flags())) { return; }

        vts::NodeInfo nodeInfo(rf, tid);
        // filter out by SDS SRS if asked to
        if (optSrs_ && (nodeInfo.srs() != optSrs_)) { return; }

        std::cout << tid << '\n';
        std::cout << "    flags: " << vts::MetaFlags(node.flags())
                  << '\n';

        std::cout
            << "    tileindex flags: " << vts::TileFlags(tiFlags) << '\n';

        std::cout << "    SDS srs: " << nodeInfo.srs() << '\n';
        std::cout << "    SDS extents: " << nodeInfo.extents() << '\n';
        std::cout << "    extents: " << node.extents << '\n';
        std::cout << "    geomExtents: " << node.geomExtents << '\n';
        if (const auto itc = node.internalTextureCount()) {
            std::cout
                << "    texture count: " << itc
                << '\n';
        }

        if (node.applyTexelSize()) {
            std::cout << "    texel size: " << node.texelSize << '\n';
        }
        if (node.applyDisplaySize()) {
            std::cout << "    display size: " << node.displaySize
                      << '\n';
        }
        if (node.navtile()) {
            std::cout << "    height range: " << node.heightRange << '\n';
        }
        if (node.sourceReference) {
            std::cout
                << "    source reference: " << node.sourceReference << '\n';
        }

        showCredits(std::cout, node, "    ");

        std::cout << "    children:" << '\n';
        for (const auto &childId : children(node, tid)) {
            std::cout << "        " << childId << '\n';
            metaChildIds.insert(ts.metaId(childId));
        }
        std::cout << '\n';
    });

    // write meta tile children
    if (!metaChildIds.empty()) {
        std::cout << "Metatile children:\n";
        for (const auto &metaChildId : metaChildIds) {
            std::cout << "    " << metaChildId << '\n';
        }
    }

    // done
    std::cout.flush();

    return EXIT_SUCCESS;
}

int VtsStorage::dumpMetatileFile()
{
    std::cout << std::fixed;

    auto meta(vts::loadMetaTile(path_, referenceFrame_.metaBinaryOrder));

    std::cout << "Metatile ID: " << meta.origin() << '\n';

    // FIXME: ensure we have meta id
    std::cout << "Parent metatile ID: "
              << vts::parent(meta.origin()) << '\n';

    {
        auto e(meta.validExtents());
        std::cout << "Covered global tile extents: " << e << '\n';
        if (!math::valid(e)) {
            std::cout << "No valid tile in this metatile." << '\n';
            return EXIT_SUCCESS;
        }
    }

    std::set<vts::TileId> metaChildIds;

    meta.for_each([&](const vts::TileId &tid, const vts::MetaNode &node)
    {
        // filter out nodes with content not needed
        if (metaFlags_.value && !(metaFlags_.value & node.flags())) { return; }

        vts::NodeInfo nodeInfo(referenceFrame_, tid);
        // filter out by SDS SRS if asked to
        if (optSrs_ && (nodeInfo.srs() != optSrs_)) { return; }

        std::cout << tid << '\n';
        std::cout << "    flags: " << vts::MetaFlags(node.flags())
                  << '\n';

        std::cout << "    SDS srs: " << nodeInfo.srs() << '\n';
        std::cout << "    SDS extents: " << nodeInfo.extents() << '\n';
        std::cout << "    extents: " << node.extents << '\n';
        std::cout << "    geomExtents: " << node.geomExtents << '\n';
        if (const auto itc = node.internalTextureCount()) {
            std::cout
                << "    texture count: " << itc
                << '\n';
        }

        if (node.applyTexelSize()) {
            std::cout << "    texel size: " << node.texelSize << '\n';
        }
        if (node.applyDisplaySize()) {
            std::cout << "    display size: " << node.displaySize
                      << '\n';
        }
        if (node.navtile()) {
            std::cout << "    height range: " << node.heightRange << '\n';
        }
        if (node.sourceReference) {
            std::cout
                << "    source reference: " << node.sourceReference << '\n';
        }

        showCredits(std::cout, node, "    ");

        std::cout << "    children:" << '\n';
        for (const auto &childId : children(node, tid)) {
            std::cout << "        " << childId << '\n';
            // FIXME: use meta id
            metaChildIds.insert(childId);
        }
        std::cout << '\n';
    });

    // write meta tile children
    if (!metaChildIds.empty()) {
        std::cout << "Metatile children:\n";
        for (const auto &metaChildId : metaChildIds) {
            std::cout << "    " << metaChildId << '\n';
        }
    }

    // done
    std::cout.flush();

    return EXIT_SUCCESS;
}

int VtsStorage::metatileVersion()
{
    std::cout << std::fixed;

    auto ts(vts::openTileSet(path_));
    std::cout << "Version: " << ts.getMetaTileVersion(tileId_) << '\n';

    return EXIT_SUCCESS;
}

int VtsStorage::mapConfig()
{
    switch (vts::datasetType(path_)) {
    case vts::DatasetType::TileSet:
        saveMapConfig(vts::TileSet::mapConfig(path_), std::cout);
        return EXIT_SUCCESS;

    case vts::DatasetType::Storage:
        saveMapConfig(vts::Storage::mapConfig(path_), std::cout, &mco_);
        return EXIT_SUCCESS;

    case vts::DatasetType::StorageView:
        saveMapConfig(vts::StorageView::mapConfig(path_), std::cout, &mco_);
        return EXIT_SUCCESS;

    default: break;
    }
    std::cerr << "Path " << path_ << " cannot produce any mapConfig."
              << '\n';
    return EXIT_FAILURE;
}

int VtsStorage::dirs()
{
    switch (vts::datasetType(path_)) {
    case vts::DatasetType::TileSet:
        saveDirs(vts::TileSet::mapConfig(path_), std::cout);
        return EXIT_SUCCESS;

    case vts::DatasetType::Storage:
        saveDirs(vts::Storage::mapConfig(path_), std::cout, &mco_);
        return EXIT_SUCCESS;

    case vts::DatasetType::StorageView:
        saveDirs(vts::StorageView::mapConfig(path_), std::cout, &mco_);
        return EXIT_SUCCESS;

    default: break;
    }
    std::cerr << "Path " << path_ << " cannot produce any directory listing."
              << '\n';
    return EXIT_FAILURE;
}

int VtsStorage::dumpTileIndex()
{
    auto ts(vts::openTileSet(path_));
    dumpAsImages(outputPath_, ts.tileIndex(), tileFlags_);
    return EXIT_SUCCESS;
}

int VtsStorage::tileInfo()
{
    std::cout << std::fixed;

    auto ts(vts::openTileSet(path_));

    const auto *n(ts.getMetaNode(tileId_, std::nothrow));
    if (!n) {
        std::cerr << tileId_ << ": no such tile" << '\n';
        return EXIT_FAILURE;
    }

    auto flags(ts.tileIndex().get(tileId_));
    std::cout << tileId_ << ": " << vts::TileFlags(flags) << '\n';
    vts::NodeInfo ni(ts.referenceFrame(), tileId_);

    std::cout
        << "Node info:"
        << "\n    parent: " << vts::parent(tileId_)
        << "\n    extents: " << ni.extents()
        << "\n    srs: " << ni.srs()
        << '\n';

    const auto &node(*n);

    std::cout << "Meta node:" << '\n';
    std::cout << "    flags: " << vts::MetaFlags(node.flags())
              << '\n';
    std::cout << "    extents: " << node.extents << '\n';
    std::cout << "    geomExtents: " << node.geomExtents << '\n';
    if (const auto itc = node.internalTextureCount()) {
        std::cout
            << "    texture count: " << itc
            << '\n';
    }

    if (node.applyTexelSize()) {
        std::cout << "    texel size: " << node.texelSize << '\n';
    }
    if (node.applyDisplaySize()) {
        std::cout << "    display size: " << node.displaySize << '\n';
    }
    if (node.navtile()) {
        std::cout << "    height range: " << node.heightRange << '\n';
    }

    if (node.sourceReference) {
        std::cout
            << "    source reference: " << node.sourceReference
            << '\n';
    }

    showCredits(std::cout, node, "    ");

    std::cout << "    children:" << '\n';
    for (const auto &childId : children(node, tileId_)) {
        std::cout << "        " << childId << '\n';
    }

    if (flags & vts::TileIndex::Flag::mesh) {
        auto mesh(ts.getMesh(tileId_));

        auto covered(double(100 * mesh.coverageMask.count())
                     / area(vts::Mesh::coverageSize()));

        std::cout
            << "Mesh:"
            << "\n    Submeshes: " << mesh.submeshes.size()
            << "\n    Covered: " << covered << " %"
            << '\n';

        int index(0);
        for (const auto &sm : mesh.submeshes) {
            auto ma(area(sm));
            std::cout
                << "    " << index << ":"
                << "\n        boundingBox: "
                << math::computeExtents(sm.vertices)
                << "\n        vertices: " << sm.vertices.size()
                << "\n        tc: " << sm.tc.size()
                << "\n        etc: " << sm.etc.size()
                << "\n        faces: " << sm.faces.size()
                << "\n        facesTc: " << sm.facesTc.size()
                << "\n        textureMode: " << sm.textureMode
                << "\n        area: " << ma.mesh
                << "\n        internalTextureArea: " << ma.internalTexture
                << "\n        externalTextureArea: " << ma.externalTexture
                << "\n        surfaceReference: " << int(sm.surfaceReference)
                ;

            if (sm.textureLayer) {
                std::cout
                    << "\n        textureLayer: " << *sm.textureLayer;
            }

            std::cout << '\n';
            ++index;
        }
    }

    if (flags & vts::TileIndex::Flag::atlas) {
        vts::RawAtlas atlas;
        ts.getAtlas(tileId_, atlas);

        std::cout
            << "Atlas:"
            << "\n    Textures: " << atlas.size()
            << '\n';
        for (std::size_t index(0), end(atlas.size()); index != end; ++index) {
            std::cout
                << "    " << index << ":"
                << "\n        imageSize: " << atlas.imageSize(index)
                << '\n';
        }
    }

    return EXIT_SUCCESS;
}

void printTileStatus(vts::TileSet &ts, const vts::TileId &tileId)
{
    std::cout
        << "tileId=" << tileId
        << " metaTileId=" << ts.metaId(tileId)
        << " flags=" << vts::TileFlags(ts.tileIndex().get(tileId))
        << '\n';
}

int VtsStorage::tileStatus()
{
    std::cout << std::fixed;
    auto ts(vts::openTileSet(path_));

    if (tileIds_.empty()) {
        vts::TileId tileId;
        for (;;) {
            if (!(std::cin >> tileId)) {
                if (std::cin.eof()) { break; }

                std::cin.clear();
                std::cin.ignore(1);
                LOG(err2) << "Invalid input.";
                continue;
            }

            printTileStatus(ts, tileId);
        }
    } else {
        for (const auto tileId : tileIds_) {
            printTileStatus(ts, tileId);
        }
    }

    return EXIT_SUCCESS;
}

int VtsStorage::dumpMesh()
{
    auto ts(vts::openTileSet(path_));

    if (!(ts.tileIndex().get(tileId_) & vts::TileIndex::Flag::mesh)) {
        std::cerr << tileId_ << ": has no mesh" << '\n';
        return EXIT_FAILURE;
    }

    auto mesh(ts.getMesh(tileId_));

    int index(0);
    for (const auto &sm : mesh.submeshes) {
        std::cout << "submesh[" << index << "] ("
                  << sm.vertices.size() << " vertices):" << '\n';

        const auto &v(sm.vertices);

        std::cout << "faces[" << index << "] ("
                  << sm.faces.size() << " faces):" << '\n';
        for (const auto &f : sm.faces) {
            std::cout
                << std::fixed << "    " << f(0) << ", " << f(1) << ", " << f(2)
                << std::flush;
            std::cout
                << " -> " << v[f(0)] << ", " << v[f(1)]
                << ", " << v[f(2)] << "\n";
        }

        if (!sm.tc.empty()) {
            std::cout << "\ntexture faces[" << index << "]" << '\n';
            for (const auto &f : sm.facesTc) {
                std::cout
                    << std::fixed << "    " << f(0) << ", "
                    << f(1) << ", " << f(2)
                    << " -> " << sm.tc[f(0)] << ", " << sm.tc[f(1)]
                    << ", " << sm.tc[f(2)] << "\n";
            }
        }

        if (!sm.etc.empty()) {
            std::cout << "\nexternal texture faces[" << index << "]"
                      << '\n';
            for (const auto &f : sm.faces) {
                std::cout
                    << std::fixed << "    " << f(0) << ", "
                    << f(1) << ", " << f(2)
                    << " -> " << sm.etc[f(0)] << ", " << sm.etc[f(1)]
                    << ", " << sm.etc[f(2)] << "\n";
            }
        }

        ++index;
    }

    return EXIT_SUCCESS;
}

int VtsStorage::dumpMeshMask()
{
    auto ts(vts::openTileSet(path_));

    if (!(ts.tileIndex().get(tileId_) & vts::TileIndex::Flag::mesh)) {
        std::cerr << tileId_ << ": has no mesh\n";
        return EXIT_FAILURE;
    }

    imgproc::png::write(outputPath_
                        , vts::debugMask(ts.getMeshMask(tileId_, generate_))
                        , 9);

    return EXIT_SUCCESS;
}

void dumpMT(const std::string &what, const vts::multifile::Table &mt)
{
    std::cout << what << ": version=" << mt.version << ", magic=\""
              << mt.magic << "\", entries:\n";
    for (const auto &e : mt.entries) {
        std::cout << "    " << e.start << ", " << e.size << "\n";
    }
}

int VtsStorage::tileMultifileLayout()
{
    auto ts(vts::openTileSet(path_));
    auto flags(ts.tileIndex().get(tileId_));

    if (!flags) {
        std::cerr << tileId_ << ": no such tile\n";
        return EXIT_FAILURE;
    }

    flags &= tileFlags_.value;

    const auto &driver(ts.driver());

    if (flags & vts::TileIndex::Flag::mesh) {
        dumpMT("mesh", vts::readMeshTable
               (driver.input(tileId_, vs::TileFile::mesh)));
    }

    if (flags & vts::TileIndex::Flag::atlas) {
        dumpMT("atlas", vts::Atlas::readTable
               (driver.input(tileId_, vs::TileFile::atlas)));
    }

    if (flags & vts::TileIndex::Flag::navtile) {
        dumpMT("navtile", vts::NavTile::readTable
               (driver.input(tileId_, vs::TileFile::navtile)));
    }

    return EXIT_SUCCESS;
}

int VtsStorage::exportMesh()
{
    auto ts(vts::openTileSet(path_));

    const auto flags(ts.tileIndex().get(tileId_));

    if (!(flags & vts::TileIndex::Flag::mesh)) {
        std::cerr << tileId_ << ": has no mesh" << '\n';
        return EXIT_FAILURE;
    }

    auto mesh(ts.getMesh(tileId_));

    {
        const auto &rf(ts.referenceFrame());
        const vts::NodeInfo ni(rf, tileId_);
        const vts::CsConvertor conv(rf.model.physicalSrs, ni.srs());
        const auto trafo(vts::MeshOpInput::sd2Coverage
                         (ni, addOptions_.safetyMargin));

        switch (meshFormat_) {
        case MeshFormat::geo: break;

        case MeshFormat::normalized: {
            const auto c(math::center(vts::extents(mesh)));
            for (auto &sm : mesh.submeshes) {
                for (auto &v : sm.vertices) {
                    v -= c;
                }
            }
        } break;

        case MeshFormat::sds:
            for (auto &sm : mesh.submeshes) {
                for (auto &v : sm.vertices) {
                    v = conv(v);
                }
            }
            break;

        case MeshFormat::sdsNormalized: {
            const auto c(math::center(ni.extents()));
            for (auto &sm : mesh.submeshes) {
                for (auto &v : sm.vertices) {
                    v = conv(v);
                    v(0) -= c(0);
                    v(1) -= c(1);
                }
            }
        } break;

        case MeshFormat::coverage:
            for (auto &sm : mesh.submeshes) {
                for (auto &v : sm.vertices) {
                    v = transform(trafo, conv(v));
                }
            }
            break;
        }
    }

    boost::optional<vts::RawAtlas> atlas;
    if (flags & vts::TileIndex::Flag::atlas) {
        atlas = boost::in_place();
        ts.getAtlas(tileId_, *atlas);
    }

    fs::create_directories(outputPath_);

    std::string mtlLib;

    if (atlas) {
        mtlLib = "textures.mtl";

        std::ofstream f((outputPath_ / mtlLib).string());

        int index(0);
        for (const auto &image : atlas->get()) {
            const auto textureName(str(boost::format("%s.jpg") % index));
            utility::write(outputPath_ / textureName
                           , image.data(), image.size());
            f << "newmtl " << index
              << "\nmap_Kd " << textureName
              << "\n";

            ++index;
        }

        f.close();
    }

    int index(0);
    for (const auto &sm : mesh.submeshes) {
        const auto objName(str(boost::format("%s.obj") % index));

        vts::saveSubMeshAsObj(outputPath_ / objName, sm
                              , index, atlas ? &*atlas : nullptr
                              , mtlLib);

        ++index;
    }

    return EXIT_SUCCESS;
}

int VtsStorage::tileIndexInfo()
{
    vts::TileIndex ti;
    ti.load(path_);
    std::cout << "lodRange: " << ti.lodRange() << '\n';

    tiInfo(ti);

    for (const auto &tileId : tileIds_) {
        auto flags(ti.get(tileId));
        std::cout << tileId << ": " << vts::TileFlags(flags) << '\n';
    }

    return EXIT_SUCCESS;
}

int VtsStorage::tileIndexRanges()
{
    vts::TileIndex ti;
    ti.load(path_);

    auto stat(ti.statMask(tileFlags_.value, tileFlags_.value));

    if (stat.lodRange.empty()) {
        return EXIT_SUCCESS;
    }

    if (verbose_) {
        const auto ts(vts::openTileSet(path_.parent_path()));
        const auto& rf(ts.referenceFrame());

        // for each srs
        for (const auto& node : vts::NodeInfo::nodes(rf)) {
            auto lod(stat.lodRange.min);

            if (node.nodeId().lod > lod) {
                continue;
            }

            vts::LodRange lodRange;
            vts::TileRange lastTr;
            auto anyTiles(false);

            for (const auto &tr : stat.tileRanges) {

                // get node infos for extremes
                const auto trNodeLl(vts::NodeInfo(rf
                    , vts::TileId(lod, tr.ll(0), tr.ll(1))
                    , false));

                const auto trNodeUr(vts::NodeInfo(rf
                    , vts::TileId(lod, tr.ur(0), tr.ur(1))
                    , false));

                // if either extreme belongs to current node subtree
                if (vts::compatible(node, trNodeLl)
                    || vts::compatible(node, trNodeUr)) {

                    auto chr(vts::childRange(node.nodeId(), lod));
                    if (valid(tr) && vts::tileRangesOverlap(tr, chr)) {
                        lastTr = vts::tileRangesIntersect(tr, chr);
                        // update lod range
                        if (!anyTiles) {
                            lodRange.min = lod;
                            anyTiles = true;
                        }
                        lodRange.max = lod;
                    }
                }
                ++lod;
            }

            // print srs with its lod range and tilerange on finest lod
            if (anyTiles) {
                std::cout << "range<" << node.srs() << ">: " << lodRange << ' '
                          << lodRange.max <<  '/' << lastTr << '\n';
            }
        }
    } else {
        auto lod(stat.lodRange.min);
        for (const auto &tr : stat.tileRanges) {
            if (valid(tr)) {
                std::cout << lod << '/' << tr << '\n';
            }
            ++lod;
        }
    }

    return EXIT_SUCCESS;
}

int VtsStorage::convertTileIndex()
{
    vts::TileIndex ti;
    ti.load(path_);
    ti.save(outputPath_);

    return EXIT_SUCCESS;
}

int VtsStorage::concat()
{
    vts::CloneOptions createOptions;
    createOptions.tilesetId(optTilesetId_);
    createOptions.lodRange(optLodRange_);
    createOptions.mode(createMode_);

    vts::concatTileSets(path_, tilesets_, createOptions);

    LOG(info4) << "All done.";
    return EXIT_SUCCESS;
}

template <typename F>
int checkForPendingError(const F &f, const std::string &what)
{
    try {
        f();
    } catch (const vts::PendingGluesError &pg) {
        std::cerr << "vts: cannot " << what << ", pending glues:\n";
        for (const auto &glue : pg.glues()) {
            std::cerr << "    <" << utility::join(glue, ",") << ">\n";
        }
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}

int VtsStorage::aggregate()
{
    vts::CloneOptions createOptions;
    createOptions.tilesetId(optTilesetId_);
    createOptions.mode(createMode_);
    createOptions.lodRange(optLodRange_);

    switch (vts::datasetType(path_)) {
    case vts::DatasetType::Storage:
        if (tilesetIds_.empty()) {
            throw po::validation_error
                (po::validation_error::invalid_option_value
                 , "tileset");
        }

        return checkForPendingError([&]() {
            vts::aggregateTileSets(outputPath_, vts::openStorage(path_)
                                   , createOptions, tilesetIds_);
        }, "aggregate");

    case vts::DatasetType::StorageView:
        if (!tilesetIds_.empty()) {
            throw po::validation_error
                (po::validation_error::invalid_option_value
                 , "tileset");
        }

        return checkForPendingError([&]() {
            vts::aggregateTileSets(outputPath_, vts::openStorageView(path_)
                                   , createOptions);
        }, "aggregate");

    default: break;;
    }

    std::cerr << "Cannot aggregate " << path_ << "." << '\n';
    return EXIT_FAILURE;
}

int VtsStorage::remote()
{
    vts::CloneOptions createOptions;
    createOptions.tilesetId(optTilesetId_);
    createOptions.mode(createMode_);

    vts::createRemoteTileSet(path_, remoteUrl_, createOptions);
    return EXIT_SUCCESS;
}

int VtsStorage::local()
{
    vts::CloneOptions createOptions;
    createOptions.tilesetId(optTilesetId_);
    createOptions.mode(createMode_);

    vts::createLocalTileSet(path_, localPath_, createOptions);
    return EXIT_SUCCESS;
}

int VtsStorage::clone()
{
    vts::CloneOptions cloneOptions;
    cloneOptions
        .tilesetId(optTilesetId_)
        .lodRange(optLodRange_)
        .mode(createMode_)
        .encodeFlags(encodeFlags_.value)
        .textureQuality(textureQuality_)
        .sameType(sameType_)
        ;

    if (!forceCredits_.empty()) {
        cloneOptions.metaNodeManipulator(
            [&](vts::MetaNode metanode) -> vts::MetaNode {
                metanode.setCredits(forceCredits_);
                return metanode;
            });
    }

    switch (vts::datasetType(path_)) {
    case vts::DatasetType::TileSet:
        vts::cloneTileSet
            (tileset_, vts::openTileSet(path_), cloneOptions);

        LOG(info4) << "All done.";
        return EXIT_FAILURE;

    case vts::DatasetType::Storage:
        vts::Storage(path_, vts::OpenMode::readOnly)
            .clone(tileset_, cloneOptions);

        LOG(info4) << "All done.";
        return EXIT_SUCCESS;

    case vts::DatasetType::StorageView:
        vts::StorageView(path_).clone(tileset_, cloneOptions);

        LOG(info4) << "All done.";
        return EXIT_SUCCESS;

    default: break;
    }

    std::cerr << "Unrecognized content " << path_ << "." << '\n';
    return EXIT_FAILURE;
}

int VtsStorage::tilePick()
{
    auto its(vts::openTileSet(path_));
    auto props(its.getProperties());

    auto ots(vts::createTileSet(tileset_, props, createMode_));

    for (const auto &tileId : tileIds_) {
        ots.setTile(tileId, its.getTileSource(tileId));
    }

    ots.flush();

    return EXIT_SUCCESS;
}

int VtsStorage::relocate()
{
    switch (vts::datasetType(path_)) {
    case vts::DatasetType::TileSet:
        vts::TileSet::relocate(path_, relocateOptions_);
        return EXIT_SUCCESS;

    case vts::DatasetType::Storage:
        vts::Storage::relocate(path_, relocateOptions_);
        return EXIT_SUCCESS;

    case vts::DatasetType::StorageView:
        vts::StorageView::relocate(path_, relocateOptions_);
        return EXIT_SUCCESS;

    default: break;
    }

    std::cerr << "Unrecognized content " << path_ << "." << '\n';
    return EXIT_FAILURE;
}

int VtsStorage::reencode()
{
    auto ro(reencodeOptions_);
    ro.encodeFlags = encodeFlags_.value;

    switch (vts::datasetType(path_)) {
    case vts::DatasetType::TileSet:
        vts::TileSet::reencode(path_, ro);
        return EXIT_SUCCESS;

    case vts::DatasetType::Storage:
        vts::Storage::reencode(path_, ro);
        return EXIT_SUCCESS;

    case vts::DatasetType::StorageView:
        vts::StorageView::reencode(path_, ro);
        return EXIT_SUCCESS;

    default: break;
    }

    std::cerr << "Unrecognized content " << path_ << "." << '\n';
    return EXIT_FAILURE;
}

int VtsStorage::reencodeCleanup()
{
    auto ro(reencodeOptions_);
    ro.cleanup = true;

    switch (vts::datasetType(path_)) {
    case vts::DatasetType::TileSet:
        vts::TileSet::reencode(path_, ro);
        return EXIT_SUCCESS;

    case vts::DatasetType::Storage:
        vts::Storage::reencode(path_, ro);
        return EXIT_SUCCESS;

    case vts::DatasetType::StorageView:
        vts::StorageView::reencode(path_, ro);
        return EXIT_SUCCESS;

    default: break;
    }

    std::cerr << "Unrecognized content " << path_ << "." << '\n';
    return EXIT_FAILURE;
}

int serveFile(const vts::Delivery::pointer &delivery
              , const std::string &filename)
{
    vts::TileId tileId;
    vts::TileFile type;
    unsigned int subTileIndex;
    vts::FileFlavor flavor;

    if (!vts::fromFilename(tileId, type, subTileIndex, filename, 0, &flavor)) {
        std::cerr << "Unrecognized filename " << filename << "." << '\n';
        return EXIT_FAILURE;
    }

    // open file
    auto is(delivery->input(tileId, type, flavor));

    // copy file to stdout
    copyFile(is, std::cout);
    return EXIT_SUCCESS;
}

int VtsStorage::file()
{
    auto root(path_.parent_path());
    auto file(path_.filename());

    switch (vts::datasetType(root)) {
    case vts::DatasetType::TileSet:
        return serveFile(vts::Delivery::open(root), file.string());

    case vts::DatasetType::Storage:
    case vts::DatasetType::StorageView:
    default: break;
    }

    std::cerr << "Unrecognized content " << path_ << "." << '\n';
    return EXIT_FAILURE;
}

int VtsStorage::glueRulesSyntax()
{
    std::cout <<
        R"RAW(Syntax of glue.rules file

glue.rules file is used to limit number of generated glues between tileset
inside a storage in a consistent way. Each tileset in the storage can be
assigned number of tags.

When adding tileset to the storage rules are consulted whether glue can be
generated for each considered combination of glues. If any rule reports
failure glue is not generated. So far, glues check only tags.

There are these rules available:

 * tag.sole-occurrence(TAG)

    Rule checks whether given TAG is present in all tileset at most once. If TAG
    is present more than once this rule fails and glue is not generated.

    TAG is any combination of characters in [-_.a-zA-Z0-9*?].

    Example: tag.sole-occurrence(world)
    Effect: only one tileset with tag "world" can exist in any glue.


 * tag.common-match(GLOB-PATTERN)

    Rule matches all tags in all tilesets agains GLOB-PATTERN (the same way as
    shell processes wildcards). Rule fails if there are more tags
    that match given pattern.

    GLOB-PATTERN is any combination of characters in [-_.a-zA-Z0-9*?], * and ?
    are interpreted as shell wildcards.

    Example: tag.common-match(user.*)
    Effect: if tags of tilesets contain tag user.XXX and user.YYY then
            glue is not generated; if there is only user.XXX or no user.*
            at all glue is generated.

 * tag.no-glue(TAG)

   If any tileset contains given TAG no glue will be generated because that
   tileset doesn't like to make any glue.

   Example: tag.no-glue(no-glue)
   Effect: no glues will be generated for tilesets having tag "no-glue".

)RAW";
    return EXIT_SUCCESS;
}

int VtsStorage::mergeConfSyntax()
{
    std::cout <<
        R"RAW(Syntax of merge.conf file

merge.conf file to specify tileset open and glue create options for add
operations.

merge.conf is regular INI config file.

Supported options:

CNAME: Maps hostname stored inside remote tileset URL to another hostname during
       merge. Can be used to fetch data from local machine instead of from
       global CDN with longer round trip.

       Syntax example:

           [cname]
           cds.melown.com = cdn-source.melown.com

       effect: all URLs that point to cdn.melown.com are rewritten to
       cdn-source.melown.com before resource is fetched.

)RAW";
    return EXIT_SUCCESS;
}

int VtsStorage::dumpNavtile()
{
    auto ts(vts::openTileSet(path_));

    if (!(ts.tileIndex().get(tileId_) & vts::TileIndex::Flag::navtile)) {
        std::cerr << tileId_ << ": has no navtile" << '\n';
        return EXIT_FAILURE;
    }

    vts::RawNavTile navtile;
    ts.getNavTile(tileId_, navtile);
    const auto &image(navtile.get());

    utility::write(outputPath_, image.data(), image.size());

    return EXIT_SUCCESS;
}

int VtsStorage::dumpNavtileMask()
{
    auto ts(vts::openTileSet(path_));

    if (!(ts.tileIndex().get(tileId_) & vts::TileIndex::Flag::navtile)) {
        std::cerr << tileId_ << ": has no navtile" << '\n';
        return EXIT_FAILURE;
    }

    vts::RawNavTile navtile;
    ts.getNavTile(tileId_, navtile);

    auto coverage(asCvMat(navtile.coverageMask()));

    create_directories(outputPath_.parent_path());
    imwrite(outputPath_.string(), coverage);

    return EXIT_SUCCESS;
}

namespace {

math::Extents2 extentsPlusHalfPixel(const math::Extents2 &extents
                                    , const math::Size2 &pixels)
{
    auto es(math::size(extents));
    const math::Size2f px(es.width / pixels.width, es.height / pixels.height);
    const math::Point2 hpx(px.width / 2, px.height / 2);
    return math::Extents2(extents.ll - hpx, extents.ur + hpx);
}

} // namespace

int VtsStorage::navtile2dem()
{
    auto ts(vts::openTileSet(path_));

    if (!(ts.tileIndex().get(tileId_) & vts::TileIndex::Flag::navtile)) {
        std::cerr << tileId_ << ": has no navtile" << '\n';
        return EXIT_FAILURE;
    }

    vts::NodeInfo nodeInfo(ts.referenceFrame(), tileId_);

    vts::opencv::NavTile navtile;
    ts.getNavTile(tileId_, navtile);

    auto size(vts::NavTile::size());

    create_directories(outputPath_.parent_path());
    auto ds(geo::GeoDataset::create
            (outputPath_, nodeInfo.srsDef()
             , extentsPlusHalfPixel(nodeInfo.extents(), size), size
             , geo::GeoDataset::Format::dsm()
             , geo::GeoDataset::NodataValue(-1e6)));

    // convert navtile to 64 bit matrix
    cv::Mat tmp;
    navtile.data().convertTo(tmp, CV_64F);
    // assign data
    ds.data() = tmp;

    // set mask
    ds.mask() = navtile.coverageMask();

    // flush
    ds.flush();

    return EXIT_SUCCESS;
}

int VtsStorage::showLockerApi()
{
    std::cout <<
        R"RAW(Eexternal locking program interface

PREREQUISITES

Locking takes effect only iff user runs one of these commands:
    --add
    --glue-generate*
    --remove
    --vs-create
    --vs-remove
and the storage contains executable file `locker`.

NB: If `locker` exists but is not executable it is silently ignored.

NB: It is an error to run vts tool with --lock paramter while `locker` is not
    available.

API

`locker` program is run in a child process with stdin and stdout connected to
pipes from main (vts) process.

First commandline argument is set either to an empty string or to value of
--lock parameter.

Stdin pipe is never written to and is closed automatically when the vts process
is terminated. This is the way how vts process termination is reported to the
locking process.

During startup phase vts process expect locking process to try to acquire a lock
(or use provided lock). If lock acquisition succeeds locking process must write
single line "OK\n" which signals vts process to go on.

If lock acquisition fails locker process should write single short line
describing the problem to stdout.

When locking process detects termination of vts process (via closed stdin) it
should release to lock and terminate itself.

Locking process is expected to hold lock whole time vts process is running. When
unrecoverable problem with held lock is detected during locking process lifetime
if must terminate vts process (i.e. kill its parent).

Process management:

    * vts process terminates on SIGCHLD, i.e. when locking process unexpectedly
      dies

    * locking process is moved to its own process group to be imune to terminal
      signal handling, i.e. Ctrl-C is delivered only to vts process and locking
      process can safely release the lock

)RAW";
    return EXIT_SUCCESS;
}

int VtsStorage::deriveMetaIndex()
{
    auto ts(vts::openTileSet(path_));
    ts.metaIndex().save(outputPath_, vts::TileIndex::SaveParams().bw(true));

    return EXIT_SUCCESS;
}

int VtsStorage::virtualSurfaceCreate()
{
    vts::TilesetIdSet tids;

    switch (vts::datasetType(path_)) {
    case vts::DatasetType::Storage:
        if (tilesetIds_.empty()) {
            std::cerr << "Missing tileset IDs." << std::endl;
            return EXIT_FAILURE;
        }

        tids.insert(tilesetIds_.begin(), tilesetIds_.end());
        break;

    case vts::DatasetType::StorageView:
        // FIXME: doesn't work for tagged tilesets
        if (!tilesetIds_.empty()) {
            std::cerr
                << "Cannot use tileset IDs when working with storage view."
                << std::endl;
            return EXIT_FAILURE;
        }

        {
            // open storage view and grab info
            const auto view(vts::openStorageView(path_));
            path_ = view.storagePath();
            tids = view.tilesets();
        }
        break;

    default:
        std::cerr << "Cannot create virtual surface in "
                  << path_ << "." << '\n';
        return EXIT_FAILURE;
    }

    // lock if external locking program is available
    Lock lock(path_, lock_);
    auto storage(vts::openStorage(path_, vts::OpenMode::readWrite, lock));

    return checkForPendingError([&]()
    {
        vts::CloneOptions createOptions;
        createOptions.mode(createMode_);
        createOptions.lodRange(optLodRange_);
        createOptions.tilesetId(optTilesetId_);

        storage.createVirtualSurface(tids, createOptions);
    }, "create virtual surface");
}

int VtsStorage::virtualSurfaceRemove()
{
    vts::TilesetIdSet tids;

    switch (vts::datasetType(path_)) {
    case vts::DatasetType::Storage:
        if (tilesetIds_.empty()) {
            throw po::validation_error
                (po::validation_error::invalid_option_value
                 , "tileset");
        }

        tids.insert(tilesetIds_.begin(), tilesetIds_.end());
        break;

    case vts::DatasetType::StorageView:
        // FIXME: doesn't work for tagged tilesets
        if (!tilesetIds_.empty()) {
            throw po::validation_error
                (po::validation_error::invalid_option_value
                 , "tileset");
        }

        {
            // open storage view and grab info
            const auto view(vts::openStorageView(path_));
            path_ = view.storagePath();
            tids = view.tilesets();
        }
        break;

    default:
        std::cerr << "Cannot remove virtual surface from "
                  << path_ << "." << '\n';
        return EXIT_FAILURE;
    }

    // lock if external locking program is available
    Lock lock(path_, lock_);
    auto storage(vts::openStorage(path_, vts::OpenMode::readWrite, lock));

    storage.removeVirtualSurface(tids);
    return EXIT_SUCCESS;
}

int VtsStorage::virtualSurfaceInfo()
{
    vts::TilesetIdSet tids;

    switch (vts::datasetType(path_)) {
    case vts::DatasetType::Storage:
        if (tilesetIds_.empty()) {
            throw po::validation_error
                (po::validation_error::invalid_option_value
                 , "tileset");
        }

        tids.insert(tilesetIds_.begin(), tilesetIds_.end());
        break;

    case vts::DatasetType::StorageView:
        // FIXME: doesn't work for tagged tilesets
        if (!tilesetIds_.empty()) {
            throw po::validation_error
                (po::validation_error::invalid_option_value
                 , "tileset");
        }

        {
            // open storage view and grab info
            const auto view(vts::openStorageView(path_));
            path_ = view.storagePath();
            tids = view.tilesets();
        }
        break;

    default:
        std::cerr << "Cannot show virtual surface info in "
                  << path_ << "." << '\n';
        return EXIT_FAILURE;
    }

    auto storage(vts::openStorage(path_, vts::OpenMode::readWrite));

    const auto ovs(storage.openVirtualSurface(tids));
    const auto &vsId(std::get<0>(ovs));
    const auto &vs(std::get<1>(ovs));

    const auto tsm
        (vts::deserializeTsMap
         (vs.driver().input(vts::VirtualSurface::TilesetMappingPath)->get()));

    int id(1);
    for (const auto &rl : tsm) {
        std::cout << id << ": ";

        bool first(true);
        for (const auto tsId : rl) {
            if (first) { first = false; } else { std::cout << " "; }
            std::cout << vsId[tsId];
        }

        std::cout << '\n';
        ++id;
    }

    return EXIT_SUCCESS;
}

int VtsStorage::listPendingGlues()
{
    const auto listGlues([&](const vts::Glue::IdSet &glueIds)
    {
        for (const auto &glueId : glueIds) {
            std::cout << utility::join(glueId, ",") << '\n';
        }
    });

    switch (vts::datasetType(path_)) {
    case vts::DatasetType::TileSet:
    case vts::DatasetType::TileIndex:
        std::cerr << "Only storage and storageview is supported\n";
        return EXIT_FAILURE;

    case vts::DatasetType::Storage:
        listGlues(vts::openStorage(path_).pendingGlues(nullptr));
        return EXIT_SUCCESS;

    case vts::DatasetType::StorageView:
        listGlues(vts::openStorageView(path_).pendingGlues());
        return EXIT_SUCCESS;

    default: break;
    }

    std::cerr << "Unrecognized content " << path_ << ".\n";
    return EXIT_FAILURE;
}

int VtsStorage::checkMetatileTree()
{
    const auto ts(vts::openTileSet(path_));

    std::queue<vts::TileId> q;
    q.push(ts.metaId(tileId_));

    while (!q.empty()) {
        const auto &metaId(q.front());
        std::cout << metaId << std::endl;

        const auto meta(ts.getMetaTile(metaId));

        std::set<vts::TileId> metaChildIds;
        meta.for_each([&](const vts::TileId &tid, const vts::MetaNode &node)
        {
            if (node.flags()) {
                for (const auto &childId : children(node, tid)) {
                    metaChildIds.insert(ts.metaId(childId));
                }
            }
        });

        q.pop();

        for (const auto &metaId : metaChildIds) {
            q.push(metaId);
        }
    }

    return EXIT_SUCCESS;
}

namespace {

class MultiSrsPoint {
public:
    MultiSrsPoint(const vr::ReferenceFrame &rf, const math::Point2 &point
                  , const std::string &srs)
    {
        init(rf, math::Point3(point(0), point(1), 0.0), srs);
    }

    MultiSrsPoint(const vr::ReferenceFrame &rf, const math::Point3 &point
                  , const std::string &srs)
    {
        init(rf, point, srs);
    }

    const math::Point2* inside(const vts::NodeInfo &nodeInfo) const;

private:
    void init(const vr::ReferenceFrame &rf, const math::Point3 &point
              , std::string srs);

    typedef std::map<std::string, math::Point2> PointMap;
    PointMap pm_;
};

void MultiSrsPoint::init(const vr::ReferenceFrame &rf
                         , const math::Point3 &point
                         , std::string srs)
{
    if (!srs.empty() && (srs[0] == '#')) {
        if (srs == "#navigation") {
            srs = rf.model.navigationSrs;
        } else if (srs == "#physical") {
            srs = rf.model.physicalSrs;
        } else if (srs == "#public") {
            srs = rf.model.publicSrs;
        } else {
            LOGTHROW(err2, std::runtime_error)
                << "Unknown special SRS <" << srs << ">.";
        }
    }

    for (const auto &node : vts::NodeInfo::nodes(rf)) {
        vts::CsConvertor conv(srs, node.srs());
        try {
            auto p(conv(point));
            pm_[node.srs()] = math::Point2(p(0), p(1));
        } catch (const geo::ProjectionError&) {}
    }
}

inline const math::Point2* MultiSrsPoint::inside(const vts::NodeInfo &nodeInfo)
    const
{
    auto fpm(pm_.find(nodeInfo.srs()));
    if (fpm == pm_.end()) { return nullptr; }

    if (math::inside(nodeInfo.extents(), fpm->second)) {
        return &fpm->second;
    }
    return nullptr;
}

} // namespace

int VtsStorage::queryNavtile()
{
    const auto ts(vts::openTileSet(path_));

    MultiSrsPoint msp(ts.referenceFrame(), queryPoint_.get(), "#navigation");
    boost::optional<vts::TileId> tid;
    const math::Point2 *point(nullptr);
    math::Extents2 nodeExtents;
    typedef vts::TileIndex::Flag TiFlag;

    vts::visit(ts, [&](TiFlag::value_type flags, const vts::NodeInfo &ni)
               -> bool
    {
        const auto nid(ni.nodeId());
        const auto *insidePoint(msp.inside(ni));
        if (!insidePoint) { return false; }
        if (TiFlag::check(flags, TiFlag::navtile)) {
            if (!tid || (nid.lod > tid->lod)) {
                tid = nid;
                point = insidePoint;
                nodeExtents = ni.extents();
            }
        }
        if (queryLod_ && (nid.lod == *queryLod_)) { return false; }

        return true;
    });

    if (!tid) {
        std::cerr << "No such navtile exists.\n";
        return EXIT_FAILURE;
    }

    const auto px(vts::NavTile::sds2px(*point, nodeExtents));

    vts::opencv::NavTile nt;
    ts.getNavTile(*tid, nt);
    auto height(nt.sample(px));

    std::cout << std::fixed
              << "tileId: " << *tid
              << "\npoint: " << px(0) << "," << px(1)
              << "\nheight: " << height
              << "\n";

    return EXIT_SUCCESS;
}

int VtsStorage::locker2Stresser()
{
    service::RunningUntilSignalled running;

    // lock if external locking program is available
    Lock lock(path_, lock_);
    auto storage(vts::Storage(path_, vts::OpenMode::readWrite, lock));

    storage.lockStressTest(running);
    return EXIT_SUCCESS;
}

int VtsStorage::decodeTsMap()
{
    utility::ifstreambuf is(path_.string());
    const auto tsm(vts::deserializeTsMap(is));

    int id(1);
    for (const auto &rl : tsm) {
        std::cout << id << ": " << utility::join(rl, ",", "-") << '\n';
        ++id;
    }

    return EXIT_SUCCESS;
}

int VtsStorage::listReferenceFrames()
{
    for (const auto &rf : vr::system.referenceFrames) {
        std::cout << rf.first << '\n';
    }
    std::cout << std::flush;
    return EXIT_SUCCESS;
}

int main(int argc, char *argv[])
{
    return VtsStorage()(argc, argv);
}
