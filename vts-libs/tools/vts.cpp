#include <boost/uuid/uuid_io.hpp>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/predicate.hpp>

#include <opencv2/highgui/highgui.hpp>

#include "dbglog/dbglog.hpp"

#include "utility/enum-io.hpp"
#include "utility/gccversion.hpp"
#include "utility/streams.hpp"
#include "utility/time.hpp"

#include "service/cmdline.hpp"

#include "imgproc/rastermask/cvmat.hpp"

#include "geo/geodataset.hpp"

#include "../registry/po.hpp"
#include "../vts.hpp"
#include "../vts/io.hpp"
#include "../vts/atlas.hpp"
#include "../vts/tileflags.hpp"
#include "../vts/metaflags.hpp"
#include "../vts/opencv/colors.hpp"
#include "../vts/opencv/navtile.hpp"
#include "../vts/tileset/delivery.hpp"

namespace po = boost::program_options;
namespace fs = boost::filesystem;
namespace vts = vadstena::vts;
namespace vr = vadstena::registry;
namespace vs = vadstena::storage;
namespace ba = boost::algorithm;

UTILITY_GENERATE_ENUM(Command,
                      ((info))
                      ((create))
                      ((add))
                      ((readd))
                      ((remove))
                      ((dumpMetatile)("dump-metatile"))
                      ((mapConfig)("map-config"))
                      ((dirs)("dirs"))
                      ((dumpTileIndex)("dump-tileindex"))
                      ((tileInfo)("tile-info"))
                      ((dumpMesh)("dump-mesh"))
                      ((dumpMeshMask)("dump-mesh-mask"))
                      ((tileIndexInfo)("tileindex-info"))
                      ((concat)("concat"))
                      ((aggregate)("aggregate"))
                      ((remote)("remote"))
                      ((local)("local"))
                      ((clone)("clone"))
                      ((relocate)("relocate"))
                      ((tilePick)("tile-pick"))
                      ((file)("file"))
                      ((tags)("tags"))
                      ((glueRulesSyntax)("glue-rules-syntax"))
                      ((dumpNavtile)("dump-navtile"))
                      ((dumpNavtileMask)("dump-navtile-mask"))
                      ((navtile2dem))
                      )


typedef service::UnrecognizedParser UP;

class VtsStorage : public service::Cmdline {
public:
    VtsStorage()
        : service::Cmdline("vts", BUILD_TARGET_VERSION
                           , (service::DISABLE_EXCESSIVE_LOGGING
                              | service::ENABLE_UNRECOGNIZED_OPTIONS))
        , noexcept_(false), command_(Command::info), brief_(false)
    {
        addOptions_.textureQuality = 0;
        addOptions_.bumpVersion = false;
        addOptions_.dryRun = false;
        addOptions_.generateReferences = false;
        addOptions_.clip = true;

        relocateOptions_.dryRun = false;
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

    int runCommand();

    int info();

    int create();

    int add();

    int readd();

    int remove();

    int tags();

    int dumpMetatile();

    int mapConfig();

    int dirs();

    int dumpTileIndex();

    int tileInfo();

    int dumpMesh();
    int dumpMeshMask();

    int tileIndexInfo();

    int concat();

    int aggregate();

    int remote();

    int local();

    int clone();

    int relocate();

    int tilePick();

    int file();

    int glueRulesSyntax();

    int dumpNavtile();
    int dumpNavtileMask();
    int navtile2dem();

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
    std::vector<vts::TileId> tileIds_;
    std::string remoteUrl_;
    fs::path localPath_;
    boost::optional<std::string> optSrs_;
    vts::Storage::AddOptions addOptions_;
    vts::RelocateOptions relocateOptions_;
    bool brief_;
    vs::CreditIds forceCredits_;
    vts::Tags addTags_;
    vts::Tags removeTags_;

    std::map<Command, std::shared_ptr<UP> > commandParsers_;
};

namespace {

void getTags(vts::Tags &tags, const po::variables_map &vars
             , const std::string &option)
{
    if (vars.count(option)) {
        const auto &value(vars[option].as<std::vector<std::string>>());
        tags.insert(value.begin(), value.end());
    }
}

} // namespace

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
                 , "--command=info: show VTS storage info"
                 , [&](UP &p)
    {
        p.options.add_options()
            ("brief", "Brief output.")
            ;

        p.configure = [&](const po::variables_map &vars) {
            brief_ = vars.count("brief");
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

            ("bumpVersion", "Add dataset under new version")
            ("dryRun", "Simulate glue creation.")
            ("tmp", po::value<fs::path>()
             , "Temporary directory where to work with temporary data.")
            ("refs", "Generate glue surface references")
            ("no-clip", "Don't clip meshes by merge coverage.")
            ("addTag", po::value<std::vector<std::string>>()
             , "Set of tags (string identifiers) assigned to tileset. "
             "Glue rules (stored in user-editable file "
             "storage-path/glue.rules) are applied on tags and glues that "
             "result in incompatible combination of tags are not generated.")
            ;

        p.positional.add("tileset", 1);

        p.configure = [&](const po::variables_map &vars) {
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
                     , "above,below,top,bottom");
            }
            if (sum > 1) {
                throw po::validation_error
                    (po::validation_error::multiple_values_not_allowed
                     , "above,below,top,bottom");
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
            addOptions_.generateReferences = vars.count("refs");
            addOptions_.clip = !vars.count("no-clip");

            getTags(addOptions_.tags, vars, "addTag");
        };
    });

    createParser(cmdline, Command::readd
                 , "--command=readd: recomputes all glues of existing tileset"
                 " in the storage"
                 , [&](UP &p)
    {
        p.options.add_options()
            ("tilesetId", po::value(&tilesetId_)->required()
             , "TilesetId to work with.")
            ("textureQuality", po::value(&addOptions_.textureQuality)
             ->required()->default_value(addOptions_.textureQuality)
             , "Quality of repacked atlases. 0 means no repacking.")
            ("dryRun", "Simulate glue creation.")
            ("tmp", po::value<fs::path>()
             , "Temporary directory where to work with temporary data.")
            ("refs", "Do not generate references")
            ("no-clip", "Don't clip meshes by merge coverage.")
            ;

        p.positional.add("tilesetId", 1);

        p.configure = [&](const po::variables_map &vars) {
            addOptions_.dryRun = vars.count("dryRun");
            if (vars.count("tmp")) {
                addOptions_.tmp = vars["tmp"].as<fs::path>();
            }
            addOptions_.generateReferences = vars.count("refs");
            addOptions_.clip = !vars.count("no-clip");
        };
    });

    createParser(cmdline, Command::remove
                 , "--command=remove: removes tileset from VTS storage"
                 , [&](UP &p)
    {
        p.options.add_options()
            ("tileset", po::value(&tilesetIds_)->required()
             , "Id of tileset to remove (can be used more than once).")
            ;

        p.positional.add("tileset", -1);
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

    createParser(cmdline, Command::mapConfig
                 , "--command=map-config: dump tileset/storage[view] "
                 "map-config"
                 , [&](UP &p)
    {
        (void) p;
    });

    createParser(cmdline, Command::dirs
                 , "--command=dirs: dump tileset/storage[view] "
                 "list of directories"
                 , [&](UP &p)
    {
        (void) p;
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

    createParser(cmdline, Command::dumpMesh
                 , "--command=dump-mesh: mesh content"
                 , [&](UP &p)
    {
        p.options.add_options()
            ("tileId", po::value(&tileId_)->required()
             , "ID of tile to query.")
            ;
        p.positional.add("tileId", 1);
    });

    createParser(cmdline, Command::dumpMeshMask
                 , "--command=dump-mesh-mask: mesh mask as an image"
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
            ;
        p.positional.add("tileset", -1);

        p.configure = [&](const po::variables_map &vars) {
            if (vars.count("tilesetId")) {
                optTilesetId_ = vars["tilesetId"].as<std::string>();
            }

            createMode_ = (vars.count("overwrite")
                           ? vts::CreateMode::overwrite
                           : vts::CreateMode::failIfExists);
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
                    try {
                        credit = vr::system.credits(
                                    boost::lexical_cast<int>(value));
                    } catch (boost::bad_lexical_cast) {
                        credit = vr::system.credits(value);
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
            ("dryRun", "Simulate glue creation.")
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

        p.positional.add("tileset", 1);
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
            getTags(addTags_, vars, "addTag");
            getTags(removeTags_, vars, "removeTag");

            if (!addTags_.empty() || !removeTags_.empty()) {
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

    createParser(cmdline, Command::glueRulesSyntax
                 , "--command=glue-rule-syntax: show syntax of glue rules"
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
    } catch (boost::bad_lexical_cast) {}

    return false;
}

int VtsStorage::runCommand()
{
    switch (command_) {
    case Command::info: return info();

    // storage related stuff
    case Command::create: return create();
    case Command::add: return add();
    case Command::readd: return readd();
    case Command::remove: return remove();
    case Command::tags: return tags();

    case Command::dumpMetatile: return dumpMetatile();
    case Command::mapConfig: return mapConfig();
    case Command::dirs: return dirs();
    case Command::dumpTileIndex: return dumpTileIndex();
    case Command::tileInfo: return tileInfo();
    case Command::dumpMesh: return dumpMesh();
    case Command::dumpMeshMask: return dumpMeshMask();
    case Command::tileIndexInfo: return tileIndexInfo();
    case Command::concat: return concat();
    case Command::aggregate: return aggregate();
    case Command::remote: return remote();
    case Command::local: return local();
    case Command::clone: return clone();
    case Command::tilePick: return tilePick();
    case Command::relocate: return relocate();
    case Command::file: return file();
    case Command::glueRulesSyntax: return glueRulesSyntax();
    case Command::dumpNavtile: return dumpNavtile();
    case Command::dumpNavtileMask: return dumpNavtileMask();
    case Command::navtile2dem: return navtile2dem();
    }
    std::cerr << "vts: no operation requested" << std::endl;
    return EXIT_FAILURE;
}

int VtsStorage::run()
{
    if (noexcept_) {
        return runCommand();
    }

    try {
        return runCommand();
    } catch (const std::exception &e) {
        std::cerr << "vts: " << e.what() << std::endl;
        return EXIT_FAILURE;
    }
}

int tilesetInfo(const fs::path &path, bool brief);
int storageInfo(const fs::path &path, bool brief);
int storageViewInfo(const fs::path &path, bool brief);

void tiInfo(const vts::TileIndex &ti, const std::string &prefix = "")
{
    typedef vts::TileIndex::Flag TiFlag;
    typedef std::pair<TiFlag::value_type, TiFlag::value_type> Flags;
    for (auto flag : { Flags(TiFlag::mesh, TiFlag::mesh)
                , Flags(TiFlag::atlas, TiFlag::atlas)
                , Flags(TiFlag::navtile, TiFlag::navtile)
                , Flags(TiFlag::alien | TiFlag::mesh
                        , TiFlag::alien | TiFlag::mesh)
                , Flags(TiFlag::reference | TiFlag::mesh
                        , TiFlag::reference)
                })
    {
        auto stat(ti.statMask(flag.first, flag.second));
        std::cout
            << prefix << "    " << vts::TileFlags(flag.second)
            << ":" << std::endl
            << prefix << "        lodRange: " << stat.lodRange << std::endl
            << prefix << "        count = " << stat.count << std::endl
            ;

        // special handling for mesh: make statistics for watertight
        if (flag.first == vts::TileIndex::Flag::mesh) {
            auto wstat(ti.statMask(vts::TileIndex::Flag::watertight));
            std::cout
                << prefix << "        watertight = " << wstat.count
                << std::endl;
        }
    }
}

int tilesetInfo(const std::string &prefix, const fs::path &path
                , bool brief)
{
    auto ts(vts::openTileSet(path));
    auto prop(ts.getProperties());
    if (brief) {
        std::cout << prefix << prop.id << " [" << ts.typeInfo()
                  << "]" << std::endl;

    } else {
        std::cout << prefix << "Id: " << prop.id << std::endl;
        std::cout << prefix << "Type: " << ts.typeInfo() << std::endl;
        std::cout << prefix << "Reference frame: " << ts.referenceFrame().id
                  << std::endl;

        std::cout << prefix << "Tile type info:" << std::endl;
        tiInfo(ts.tileIndex(), prefix);
    }

    return EXIT_SUCCESS;
}

int storageInfo(const std::string &prefix, const fs::path &path, bool brief)
{
    auto s(vts::openStorage(path));
    std::cout << prefix << "Tile sets:" << std::endl;
    for (const auto &tileset : s.storedTilesets()) {
        auto subprefix(prefix + "    ");
        tilesetInfo(subprefix, s.path(tileset.tilesetId), brief);
        if (!brief) {
            if (!tileset.tags.empty()) {
                std::cout << subprefix << "Tags: "
                          << utility::join(tileset.tags, ", ") << std::endl;
            }
            std::cout << std::endl;
        }
    }

    std::cout << prefix << "Glues:" << std::endl;
    for (const auto &gitem : s.glues()) {
        if (brief) {
            std::cout << prefix << "    " << utility::join(gitem.first, ", ")
                      << std::endl;
        } else {
            std::cout << prefix << "    Glue-Id: "
                      << utility::join(gitem.first, ", ")
                      << std::endl;

            tilesetInfo(prefix + "    ",  s.path(gitem.second), brief);
            std::cout << std::endl;
        }
    }

    return EXIT_SUCCESS;
}

int storageViewInfo(const std::string &prefix, const fs::path &path
                    , bool brief)
{
    auto sv(vts::openStorageView(path));
    std::cout << prefix << "View into storage:" << std::endl
              << prefix << "    " << sv.storagePath() << std::endl;
    std::cout << prefix << "Tile sets:" << std::endl;
    for (const auto &tid : sv.tilesets()) {
        std::cout << prefix << "    " << tid << std::endl;
    }
    return EXIT_SUCCESS;
    (void) brief;
}

int VtsStorage::info()
{
    switch (vts::datasetType(path_)) {
    case vts::DatasetType::TileSet:
        return tilesetInfo(std::string(), path_, brief_);

    case vts::DatasetType::Storage:
        return storageInfo(std::string(), path_, brief_);

    case vts::DatasetType::StorageView:
        return storageViewInfo(std::string(), path_, brief_);

    default: break;
    }
    std::cerr << "Unrecognized content " << path_ << "." << std::endl;
    return EXIT_SUCCESS;
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
            || ba::istarts_with(path.string(), "https://"));
}

} // namespace

int VtsStorage::add()
{
    const auto tmpPath(path_ / "tmp/tileset-to-add");

    auto storage(vts::Storage(path_, vts::OpenMode::readWrite));

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
    return EXIT_SUCCESS;
}

int VtsStorage::readd()
{
    auto storage(vts::Storage(path_, vts::OpenMode::readWrite));
    storage.readd(tilesetId_, addOptions_);
    return EXIT_SUCCESS;
}

int VtsStorage::remove()
{
    auto storage(vts::Storage(path_, vts::OpenMode::readWrite));
    storage.remove(tilesetIds_);
    return EXIT_SUCCESS;
}

int VtsStorage::tags()
{
    auto storage(vts::Storage(path_, vts::OpenMode::readWrite));
    if (!addTags_.empty() || !removeTags_.empty()) {
        storage.updateTags(tilesetId_, addTags_, removeTags_);

        // let the info code print result
        optTilesetId_ = tilesetId_;
    }

    for (const auto &tileset : storage.storedTilesets()) {
        if (optTilesetId_ && (tileset.tilesetId != optTilesetId_)) {
            continue;
        }
        std::cout << tileset.tilesetId << ": "
                  << utility::join(tileset.tags, ", ") << std::endl;
    }

    return EXIT_SUCCESS;
}

void showCredits(std::ostream &out, const vts::MetaNode &node
                 , const std::string &prefix)
{
    out << prefix << "credits: ";
    if (node.credits().empty()) {
        out << "none" << std::endl;
    } else {
        out << node.credits().size() << std::endl;
    }

    for (const auto &credit : node.credits()) {
        out << prefix << "    " << credit;
        if (const auto *info = vr::system.credits(credit, std::nothrow)) {
            out << ": " << info->id;
        } else {
            out << ": <unknown>";
        }

        out << std::endl;
    }
    return;
}

int VtsStorage::dumpMetatile()
{
    std::cout << std::fixed;

    auto ts(vts::openTileSet(path_));
    auto rf(ts.referenceFrame());

    auto meta(ts.getMetaTile(tileId_));

    std::cout << "Metatile ID: " << meta.origin() << std::endl;

    std::cout << "Parent metatile ID: "
              << ts.metaId(vts::parent(meta.origin())) << std::endl;

    {
        auto e(meta.validExtents());
        std::cout << "Covered global tile extents: " << e << std::endl;
        if (!math::valid(e)) {
            std::cout << "No valid tile in this metatile." << std::endl;
            return EXIT_SUCCESS;
        }
    }

    meta.for_each([&](const vts::TileId &tid, const vts::MetaNode &node)
    {
        auto tiFlags(ts.tileIndex().get(tid));
        if (!tiFlags && !node.flags()) { return; }

        // filter out nodes with content not needed
        if (metaFlags_.value && !(metaFlags_.value & node.flags())) { return; }

        vts::NodeInfo nodeInfo(rf, tid);
        // filter out by SDS SRS if asked to
        if (optSrs_ && (nodeInfo.srs() != optSrs_)) { return; }

        std::cout << tid << std::endl;
        std::cout << "    flags: " << vts::MetaFlags(node.flags())
                  << std::endl;

        std::cout
            << "    tileindex flags: " << vts::TileFlags(tiFlags) << std::endl;

        std::cout << "    SDS srs: " << nodeInfo.srs() << std::endl;
        std::cout << "    SDS extents: " << nodeInfo.extents() << std::endl;
        std::cout << "    extents: " << node.extents << std::endl;
        if (node.internalTextureCount()) {
            std::cout
                << "    texture count: " << node.internalTextureCount()
                << std::endl;
        }
        if (node.reference()) {
            std::cout
                << "    reference: " << node.reference() << std::endl;
        }

        if (node.applyTexelSize()) {
            std::cout << "    texel size: " << node.texelSize << std::endl;
        }
        if (node.applyDisplaySize()) {
            std::cout << "    display size: " << node.displaySize
                      << std::endl;
        }
        if (node.navtile()) {
            std::cout << "    height range: " << node.heightRange << std::endl;
        }

        showCredits(std::cout, node, "    ");

        std::cout << "    children:" << std::endl;
        for (const auto &childId : children(node, tid)) {
            std::cout << "        " << childId << std::endl;
        }
        std::cout << std::endl;
    });

    return EXIT_SUCCESS;
}

int VtsStorage::mapConfig()
{
    switch (vts::datasetType(path_)) {
    case vts::DatasetType::TileSet:
        saveMapConfig(vts::TileSet::mapConfig(path_), std::cout);
        return EXIT_SUCCESS;

    case vts::DatasetType::Storage:
        saveMapConfig(vts::Storage::mapConfig(path_), std::cout);
        return EXIT_SUCCESS;

    case vts::DatasetType::StorageView:
        saveMapConfig(vts::StorageView::mapConfig(path_), std::cout);
        return EXIT_SUCCESS;

    default: break;
    }
    std::cerr << "Path " << path_ << " cannot produce any mapConfig."
              << std::endl;
    return EXIT_FAILURE;
}

int VtsStorage::dirs()
{
    switch (vts::datasetType(path_)) {
    case vts::DatasetType::TileSet:
        saveDirs(vts::TileSet::mapConfig(path_), std::cout);
        return EXIT_SUCCESS;

    case vts::DatasetType::Storage:
        saveDirs(vts::Storage::mapConfig(path_), std::cout);
        return EXIT_SUCCESS;

    case vts::DatasetType::StorageView:
        saveDirs(vts::StorageView::mapConfig(path_), std::cout);
        return EXIT_SUCCESS;

    default: break;
    }
    std::cerr << "Path " << path_ << " cannot produce any directory listing."
              << std::endl;
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

    auto flags(ts.tileIndex().get(tileId_));
    if (!flags) {
        std::cerr << tileId_ << ": no such tile" << std::endl;
        return EXIT_FAILURE;
    }
    std::cout << tileId_ << ": " << vts::TileFlags(flags) << std::endl;

    vts::NodeInfo ni(ts.referenceFrame(), tileId_);

    std::cout
        << "Node info:"
        << "\n    parent: " << vts::parent(tileId_)
        << "\n    extents: " << ni.extents()
        << "\n    srs: " << ni.srs()
        << std::endl;

    if (flags & (vts::TileIndex::Flag::real | vts::TileIndex::Flag::reference))
    {
        auto node(ts.getMetaNode(tileId_));

        std::cout << "Meta node:" << std::endl;
        std::cout << "    flags: " << vts::MetaFlags(node.flags())
                  << std::endl;
        std::cout << "    extents: " << node.extents << std::endl;
        if (node.internalTextureCount()) {
            std::cout
                << "    texture count: " << node.internalTextureCount()
                << std::endl;
        }
        if (node.reference()) {
            std::cout
                << "    reference: " << node.reference()
                << std::endl;
        }

        if (node.applyTexelSize()) {
            std::cout << "    texelSize: " << node.texelSize << std::endl;
        }
        if (node.applyDisplaySize()) {
            std::cout << "    displaySize: " << node.displaySize << std::endl;
        }
        if (node.navtile()) {
            std::cout << "    heightRange: " << node.heightRange << std::endl;
        }

        showCredits(std::cout, node, "    ");

        std::cout << "    children:" << std::endl;
        for (const auto &childId : children(node, tileId_)) {
            std::cout << "        " << childId << std::endl;
        }
    }

    if (flags & vts::TileIndex::Flag::mesh) {
        auto mesh(ts.getMesh(tileId_));

        auto covered(double(100 * mesh.coverageMask.count())
                     / area(vts::Mesh::coverageSize()));

        std::cout
            << "Mesh:"
            << "\n    Submeshes: " << mesh.submeshes.size()
            << "\n    Covered: " << covered << " %"
            << std::endl;

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

            std::cout << std::endl;
            ++index;
        }
    }

    if (flags & vts::TileIndex::Flag::atlas) {
        vts::RawAtlas atlas;
        ts.getAtlas(tileId_, atlas);

        std::cout
            << "Atlas:"
            << "\n    Textures: " << atlas.size()
            << std::endl;
        for (std::size_t index(0), end(atlas.size()); index != end; ++index) {
            std::cout
                << "    " << index << ":"
                << "\n        imageSize: " << atlas.imageSize(index)
                << std::endl;
        }
    }

    return EXIT_SUCCESS;
}

int VtsStorage::dumpMesh()
{
    auto ts(vts::openTileSet(path_));

    if (!(ts.tileIndex().get(tileId_) & vts::TileIndex::Flag::mesh)) {
        std::cerr << tileId_ << ": has no mesh" << std::endl;
        return EXIT_FAILURE;
    }

    auto mesh(ts.getMesh(tileId_));

    int index(0);
    for (const auto &sm : mesh.submeshes) {
        std::cout << "submesh[" << index << "]:" << std::endl;

        const auto &v(sm.vertices);

        std::cout << "faces[" << index << "]" << std::endl;
        for (const auto &f : sm.faces) {
            std::cout
                << std::fixed << "    " << f(0) << ", " << f(1) << ", " << f(2)
                << " -> " << v[f(0)] << ", " << v[f(1)]
                << ", " << v[f(2)] << "\n";
        }

        if (!sm.tc.empty()) {
            std::cout << "\ntexture faces[" << index << "]" << std::endl;
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
                      << std::endl;
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
        std::cerr << tileId_ << ": has no mesh" << std::endl;
        return EXIT_FAILURE;
    }

    auto mesh(ts.getMesh(tileId_));

    cv::Mat coverage(mesh.coverageMask.size().height
                     , mesh.coverageMask.size().width, CV_8UC3);
    coverage = cv::Scalar(0x00, 0x00, 0x00);

    mesh.coverageMask.forEachNode([&](uint xstart, uint ystart, uint size
                                      , std::uint8_t color)
    {
        cv::Point2i start(xstart, ystart);
        cv::Point2i end(xstart + size - 1, ystart + size - 1);

        cv::rectangle(coverage, start, end, vts::opencv::palette256[color]
                      , CV_FILLED, 4);
    }, vts::Mesh::CoverageMask::Filter::white);

    create_directories(outputPath_.parent_path());
    imwrite(outputPath_.string(), coverage);

    return EXIT_SUCCESS;
}

int VtsStorage::tileIndexInfo()
{
    vts::TileIndex ti;
    ti.load(path_);
    std::cout << "lodRange: " << ti.lodRange() << std::endl;

    tiInfo(ti);

    for (const auto &tileId : tileIds_) {
        auto flags(ti.get(tileId));
        std::cout << tileId << ": " << vts::TileFlags(flags) << std::endl;
    }

    return EXIT_SUCCESS;
}

int VtsStorage::concat()
{
    vts::CloneOptions createOptions;
    createOptions.tilesetId(optTilesetId_);
    createOptions.lodRange(optLodRange_);
    createOptions.mode(createMode_);

    vts::concatTileSets(path_, tilesets_, createOptions);

    return EXIT_SUCCESS;
}

int VtsStorage::aggregate()
{
    vts::CloneOptions createOptions;
    createOptions.tilesetId(optTilesetId_);
    createOptions.mode(createMode_);

    switch (vts::datasetType(path_)) {
    case vts::DatasetType::Storage:
        if (tilesetIds_.empty()) {
            throw po::validation_error
                (po::validation_error::invalid_option_value
                 , "tileset");
        }

        vts::aggregateTileSets(outputPath_, vts::openStorage(path_)
                               , createOptions, tilesetIds_);
        return EXIT_SUCCESS;

    case vts::DatasetType::StorageView:
        if (!tilesetIds_.empty()) {
            throw po::validation_error
                (po::validation_error::invalid_option_value
                 , "tileset");
        }

        vts::aggregateTileSets(outputPath_, vts::openStorageView(path_)
                               , createOptions);
        return EXIT_SUCCESS;

    default: break;;
    }

    std::cerr << "Cannot aggregate " << path_ << "." << std::endl;
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
    cloneOptions.tilesetId(optTilesetId_);
    cloneOptions.lodRange(optLodRange_);
    cloneOptions.mode(createMode_);
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
        return EXIT_FAILURE;

    case vts::DatasetType::Storage:
        vts::Storage(path_, vts::OpenMode::readOnly)
            .clone(tileset_, cloneOptions);
        return EXIT_SUCCESS;

    case vts::DatasetType::StorageView:
        vts::StorageView(path_).clone(tileset_, cloneOptions);
        return EXIT_SUCCESS;

    default: break;
    }

    std::cerr << "Unrecognized content " << path_ << "." << std::endl;
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

    std::cerr << "Unrecognized content " << path_ << "." << std::endl;
    return EXIT_FAILURE;
}

int serveFile(const vts::Delivery::pointer &delivery
              , const std::string &filename)
{
    vts::TileId tileId;
    vts::TileFile type;
    unsigned int subTileIndex;

    if (!vts::fromFilename(tileId, type, subTileIndex, filename)) {
        std::cerr << "Unrecognized filename " << filename << "." << std::endl;
        return EXIT_FAILURE;
    }

    // open file
    auto is(delivery->input(tileId, type));

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

    std::cerr << "Unrecognized content " << path_ << "." << std::endl;
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

int VtsStorage::dumpNavtile()
{
    auto ts(vts::openTileSet(path_));

    if (!(ts.tileIndex().get(tileId_) & vts::TileIndex::Flag::navtile)) {
        std::cerr << tileId_ << ": has no navtile" << std::endl;
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
        std::cerr << tileId_ << ": has no navtile" << std::endl;
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
        std::cerr << tileId_ << ": has no navtile" << std::endl;
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

int main(int argc, char *argv[])
{
    return VtsStorage()(argc, argv);
}
