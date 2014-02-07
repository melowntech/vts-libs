#include "utility/streams.hpp"

#include "./storage.hpp"
#include "./tileset.hpp"
#include "../tilestorage.hpp"
#include "./error.hpp"
#include "./json.hpp"
#include "./io.hpp"

namespace vadstena { namespace tilestorage {

namespace fs = boost::filesystem;

namespace {
    const std::string InputDir("input");

    const std::string OutputDir("output");

    const std::string DefaultInputType("flat");

    const std::string DefaultOutputType("flat");

    const std::string ConfigName("index.json");

    void saveConfigImpl(const fs::path path
                        , const StorageProperties &properties
                        , Json::Value &config)
    {
        build(config, properties);

        // save json
        try {
            std::ofstream f;
            f.exceptions(std::ifstream::failbit | std::ifstream::badbit);
            f.open(path.string());
            f.precision(15);
            Json::StyledStreamWriter().write(f, config);
            f.close();
        } catch (const std::exception &e) {
            LOGTHROW(err2, Error)
                << "Unable to write " << path << " config: "
                << e.what() << ".";
        }
    }

    Locator rooted(const fs::path &root, Locator locator) {
        // TODO: there should be probably some detection for non-fs-based
        // drivers
        locator.location = (root / locator.location).string();
        return locator;
    }
}

struct Storage::Factory
{
    static Storage::pointer create(const fs::path &root
                                   , const CreateProperties &properties
                                   , CreateMode mode)
    {
        if (!create_directories(root)) {
            // directory already exists -> fail if mode says so
            if (mode == CreateMode::failIfExists) {
                LOGTHROW(err2, StorageAlreadyExists)
                    << "Storage at " << root << " already exists.";
            }
        }

        // create input directory
        create_directories(root / InputDir);

        // create default config
        StorageProperties sp;
        sp.outputSet.locator.type = DefaultOutputType;
        sp.outputSet.locator.location = OutputDir;

        // update create properties
        CreateProperties p(properties);
        p.id = OutputDir;

        // create output tile set
        createTileSet(rooted(root, sp.outputSet.locator), p, mode);

        // save config
        Json::Value config;
        saveConfigImpl(root / ConfigName, sp, config);

        // done
        return Storage::pointer(new Storage(root, false));
    }

    static Storage::pointer open(const fs::path &root, OpenMode mode)
    {
        return Storage::pointer(new Storage(root, mode == OpenMode::readOnly));
    }
};

Storage::pointer createStorage(const fs::path &root
                               , const CreateProperties &properties
                               , CreateMode mode)
{
    return Storage::Factory::create(root, properties, mode);
}

Storage::pointer openStorage(const fs::path &root, OpenMode mode)
{
    return Storage::Factory::open(root, mode);
}

struct Storage::Detail {
    Detail(const fs::path &root, bool readOnly)
        : root(root), readOnly(readOnly)
    {}

    void loadConfig();

    void saveConfig();

    void addTileSet(const Locator &locator);

    void removeTileSet(const std::string &id);

    TileSetDescriptor& findInput(const std::string &id);

    const fs::path &root;
    Json::Value config;
    StorageProperties properties;

    bool readOnly;
};

void Storage::Detail::loadConfig()
{
    auto path(root / ConfigName);
    // load json
    try {
        std::ifstream f;
        f.exceptions(std::ifstream::failbit | std::ifstream::badbit);
        f.open(path.string());
        Json::Reader reader;
        if (!reader.parse(f, config)) {
            LOGTHROW(err2, FormatError)
                << "Unable to parse " << path << " config: "
                << reader.getFormattedErrorMessages() << ".";
        }
        f.close();
    } catch (const std::exception &e) {
        LOGTHROW(err2, Error)
            << "Unable to read " << path << " config: "
            << e.what() << ".";
    }

    parse(properties, config);
}

void Storage::Detail::saveConfig()
{
    saveConfigImpl(root / ConfigName, properties, config);
}


TileSetDescriptor& Storage::Detail::findInput(const std::string &id)
{
    auto finputSets(properties.inputSets.find(id));
    if (finputSets == properties.inputSets.end()) {
        LOGTHROW(err2, NoSuchTileSet)
            << "This storage doesn't contain input tile set <" << id << ">.";
    }
    return finputSets->second;
}

void Storage::Detail::addTileSet(const Locator &locator)
{
    auto ts(openTileSet(locator));

    LOG(info2) << "Adding tile set <" << ts->getProperties().id
               << ">:\n"
               << utility::dump(ts->getProperties(), "    ");
}

void Storage::Detail::removeTileSet(const std::string &id)
{
    auto &desc(findInput(id));

    auto ts(openTileSet(desc.locator));

    LOG(info2) << "Removing tile set:\n"
               << utility::dump(ts->getProperties(), "    ");
}

// storage

Storage::Storage(const fs::path &root, bool readOnly)
    : detail_(new Detail(root, readOnly))
{
    detail().loadConfig();
}

void Storage::addTileSet(const Locator &locator)
{
    return detail().addTileSet(locator);
}

void Storage::removeTileSet(const std::string &id)
{
    return detail().removeTileSet(id);
}

} } // namespace vadstena::tilestorage
