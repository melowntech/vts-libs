#include <boost/uuid/uuid_io.hpp>

#include <opencv2/highgui/highgui.hpp>

#include "dbglog/dbglog.hpp"

#include "utility/enum-io.hpp"
#include "utility/gccversion.hpp"
#include "utility/streams.hpp"
#include "utility/time.hpp"

#include "service/cmdline.hpp"

#include "../registry/po.hpp"
#include "../vts.hpp"
#include "../vts/io.hpp"
#include "../vts/atlas.hpp"
#include "../vts/tileflags.hpp"
#include "../vts/metaflags.hpp"

namespace po = boost::program_options;
namespace fs = boost::filesystem;
namespace vts = vadstena::vts;
namespace vr = vadstena::registry;

UTILITY_GENERATE_ENUM(Command,
                      ((info))
                      ((create))
                      ((add))
                      ((readd))
                      ((remove))
                      ((flatten))
                      ((dumpMetatile)("dump-metatile"))
                      ((mapConfig)("map-config"))
                      ((dumpTileIndex)("dump-tileindex"))
                      ((tileInfo)("tile-info"))
                      ((dumpMesh)("dump-mesh"))
                      ((dumpMeshMask)("dump-mesh-mask"))
                      ((tileIndexInfo)("tileindex-info"))
                      ((concat)("concat"))
                      ((aggregate)("aggregate"))
                      ((remote)("remote"))
                      ((local)("local"))
                      )


typedef service::UnrecognizedParser UP;

class VtsStorage : public service::Cmdline {
public:
    VtsStorage()
        : service::Cmdline("vts", BUILD_TARGET_VERSION
                           , (service::DISABLE_EXCESSIVE_LOGGING
                              | service::ENABLE_UNRECOGNIZED_OPTIONS))
        , noexcept_(false), command_(Command::info)
        , glueMode_(vts::StoredTileset::GlueMode::full)
    {}

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

    int flatten();

    int dumpMetatile();

    int mapConfig();

    int dumpTileIndex();

    int tileInfo();

    int dumpMesh();
    int dumpMeshMask();

    int tileIndexInfo();

    int concat();

    int aggregate();

    int remote();

    int local();

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
    vts::StoredTileset::GlueMode glueMode_;
    boost::optional<vts::LodRange> optLodRange_;
    fs::path outputPath_;
    vts::TileFlags tileFlags_;
    vts::MetaFlags metaFlags_;
    std::vector<vts::TileId> tileIds_;
    std::string remoteUrl_;
    fs::path localPath_;
    boost::optional<std::string> optSrs_;

    std::map<Command, std::shared_ptr<UP> > commandParsers_;
};

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
        (void) p;
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
            = vr::Registry::referenceFrame(storageProperties_.referenceFrame);
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
            ("where"
             , po::value(&where_)->required()->default_value(where_)
             , "Location of reference tileset; format: \n"
             "    @BOTTOM -- adds to the bottom of the stack"
             "    @TOP -- adds to the TOP of the stack"
             "    -tileSetId -- adds below given tileset"
             "    +tileSetId -- adds above given tileset.")
            ("glueMode", po::value(&glueMode_)->required()
             ->default_value(glueMode_)
             , "Glue generation mode.")
            ("lodRange", po::value<vts::LodRange>()
             , "Limits used LOD range from source tileset.")
            ;

        p.positional.add("tileset", 1);

        p.configure = [&](const po::variables_map &vars) {
            if (vars.count("tilesetId")) {
                optTilesetId_ = vars["tilesetId"].as<std::string>();
            }
            if (vars.count("lodRange")) {
                optLodRange_ = vars["lodRange"].as<vts::LodRange>();
            }
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
            ;

        p.positional.add("tilesetId", 1);
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

    createParser(cmdline, Command::flatten
                 , "--command=flatten: flattens contents of a VTS storage "
                 "into new tileset."
                 , [&](UP &p)
    {
        p.options.add_options()
            ("tileset", po::value(&tileset_)->required()
             , "Path to output tileset.")
            ("overwrite", "Overwrite existing output tileset.")
            ("tilesetId", po::value<std::string>()
             , "TilesetId of output tileset. Defaults to filename of "
             "tileset path ")
            ;

        p.configure = [&](const po::variables_map &vars) {
            createMode_ = (vars.count("overwrite")
                           ? vts::CreateMode::overwrite
                           : vts::CreateMode::failIfExists);
            if (vars.count("tilesetId")) {
                optTilesetId_ = vars["tilesetId"].as<std::string>();
            }
        };

        p.positional.add("tileset", 1);
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
    });

    createParser(cmdline, Command::mapConfig
                 , "--command=map-config: dump tileset/storage map-config"
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
    });

    createParser(cmdline, Command::dumpMesh
                 , "--command=dump-mesh: mesh content"
                 , [&](UP &p)
    {
        p.options.add_options()
            ("tileId", po::value(&tileId_)->required()
             , "ID of tile to query.")
            ;
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
             , "TilesetId of created tileset, defaults to last part of"
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
    case Command::create: return create();
    case Command::add: return add();
    case Command::readd: return readd();
    case Command::remove: return remove();
    case Command::flatten: return flatten();
    case Command::dumpMetatile: return dumpMetatile();
    case Command::mapConfig: return mapConfig();
    case Command::dumpTileIndex: return dumpTileIndex();
    case Command::tileInfo: return tileInfo();
    case Command::dumpMesh: return dumpMesh();
    case Command::dumpMeshMask: return dumpMeshMask();
    case Command::tileIndexInfo: return tileIndexInfo();
    case Command::concat: return concat();
    case Command::aggregate: return aggregate();
    case Command::remote: return remote();
    case Command::local: return local();
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

int tilesetInfo(const fs::path &path);
int storageInfo(const fs::path &path);
int storageViewInfo(const fs::path &path);

int tilesetInfo(const std::string &prefix, const fs::path &path)
{
    auto ts(vts::openTileSet(path));
    auto prop(ts.getProperties());
    std::cout << prefix << "Id: " << prop.id << std::endl;
    std::cout << prefix << "Type: " << ts.typeInfo() << std::endl;

    std::cout << prefix << "Tile type info:" << std::endl;
    for (auto flag : { vts::TileIndex::Flag::mesh
                , vts::TileIndex::Flag::atlas
                , vts::TileIndex::Flag::navtile
                , vts::TileIndex::Flag::reference })
    {
        auto stat(ts.tileIndex().statMask(flag));
        std::cout
            << prefix << "    " << vts::TileFlags(flag) << ":" << std::endl
            << prefix << "        lodRange: " << stat.lodRange << std::endl
            << prefix << "        count = " << stat.count << std::endl
            ;
    }

    return EXIT_SUCCESS;
}

int storageInfo(const std::string &prefix, const fs::path &path)
{
    auto s(vts::openStorage(path));
    std::cout << prefix << "Tile sets:" << std::endl;
    for (const auto &tid : s.tilesets()) {
        tilesetInfo(prefix + "    ", s.path(tid));
        std::cout << std::endl;
    }

    std::cout << prefix << "Glues:" << std::endl;
    for (const auto &gitem : s.glues()) {
        std::cout << prefix << "    Glue-Id: "
                  << utility::join(gitem.first, ", ")
                  << std::endl;

        tilesetInfo(prefix + "    ",  s.path(gitem.second));
        std::cout << std::endl;
    }

    return EXIT_SUCCESS;
}

int storageViewInfo(const std::string &prefix, const fs::path &path)
{
    auto sv(vts::openStorageView(path));
    std::cout << prefix << "View into storage:" << std::endl
              << prefix << "    " << sv.storagePath() << std::endl;
    std::cout << prefix << "Tile sets:" << std::endl;
    for (const auto &tid : sv.tilesets()) {
        std::cout << prefix << "    " << tid << std::endl;
    }
    return EXIT_SUCCESS;
}

int VtsStorage::info()
{
    switch (vts::datasetType(path_)) {
    case vts::DatasetType::TileSet:
        return tilesetInfo(std::string(), path_);

    case vts::DatasetType::Storage:
        return storageInfo(std::string(), path_);

    case vts::DatasetType::StorageView:
        return storageViewInfo(std::string(), path_);

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

int VtsStorage::add()
{
    auto storage(vts::Storage(path_, vts::OpenMode::readWrite));
    storage.add(tileset_, where_
                , vts::StoredTileset(optTilesetId_, glueMode_)
                , vts::TileFilter().lodRange(optLodRange_));
    return EXIT_SUCCESS;
}

int VtsStorage::readd()
{
    auto storage(vts::Storage(path_, vts::OpenMode::readWrite));
    storage.readd(tilesetId_);
    return EXIT_SUCCESS;
}

int VtsStorage::remove()
{
    auto storage(vts::Storage(path_, vts::OpenMode::readWrite));
    storage.remove(tilesetIds_);
    return EXIT_SUCCESS;
}

int VtsStorage::flatten()
{
    auto storage(vts::Storage(path_, vts::OpenMode::readOnly));
    storage.flatten(tileset_, createMode_);
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
        if (auto info = vr::Registry::credit(credit, std::nothrow)) {
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
                     , mesh.coverageMask.size().width, CV_8U);
    coverage = cv::Scalar(0x00);
    cv::Scalar color(0xff);
    mesh.coverageMask.forEachQuad([&](uint xstart, uint ystart, uint xsize
                                      , uint ysize, bool)
    {
        cv::Point2i start(xstart, ystart);
        cv::Point2i end(xstart + xsize - 1, ystart + ysize - 1);

        cv::rectangle(coverage, start, end, color, CV_FILLED, 4);
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

    for (auto flag : { vts::TileIndex::Flag::mesh
                , vts::TileIndex::Flag::atlas
                , vts::TileIndex::Flag::navtile
                , vts::TileIndex::Flag::reference })
    {
        auto stat(ti.statMask(flag));
        std::cout
            << "    " << vts::TileFlags(flag) << ":" << std::endl
            << "        lodRange: " << stat.lodRange << std::endl
            << "        count = " << stat.count << std::endl
            ;
    }

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

int main(int argc, char *argv[])
{
    return VtsStorage()(argc, argv);
}
