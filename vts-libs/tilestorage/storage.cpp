#include "utility/streams.hpp"

#include "./storage.hpp"
#include "./tileset.hpp"
#include "../tilestorage.hpp"
#include "./error.hpp"
#include "./json.hpp"
#include "./io.hpp"
#include "./driver.hpp"

namespace vadstena { namespace tilestorage {

namespace fs = boost::filesystem;

namespace {
    const std::string InputDir("input");

    const std::string OutputDir("output");

    const std::string DefaultInputType("flat");

    const std::string DefaultOutputType("hash/crc");

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

    typedef std::map<std::string, TileSet::pointer> TileSetMap;

    TileSet::list asList(const TileSetMap &map) {
        TileSet::list out;
        for (const auto &ts : map) {
            out.push_back(ts.second);
        }
        return out;
    }
}

struct Storage::Factory {
    static Storage::pointer create(const fs::path &root
                                   , const StorageCreateProperties &properties
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
        sp.outputSet.locator.type
            = (properties.outputTileSetType.empty()
               ? DefaultOutputType
               : properties.outputTileSetType);
        sp.outputSet.locator.location = OutputDir;

        // update create properties
        CreateProperties p(properties.createProperties);
        p.staticProperties.id = OutputDir;

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
                               , const StorageCreateProperties &properties
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

    void addTileSets(const std::vector<Locator> &locators);

    void removeTileSets(const std::vector<std::string> &ids);

    TileSetDescriptor* findInput(const std::string &id);

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


TileSetDescriptor* Storage::Detail::findInput(const std::string &id)
{
    auto finputSets(properties.inputSets.find(id));
    if (finputSets == properties.inputSets.end()) {
        return nullptr;
    }
    return &finputSets->second;
}

void Storage::Detail::addTileSets(const std::vector<Locator> &locators)
{
    TileSetMap kept;
    TileSetMap update;

    // open and add all new tile sets to the merge input
    for (const auto &locator : locators) {
        auto tileSet(openTileSet(locator));
        const auto tsp(tileSet->getProperties());

        if (findInput(tsp.id)) {
            LOGTHROW(err2, TileSetAlreadyExists)
                << "This storage already contains input tile set <"
                << tsp.id << ">.";
        }

        update.insert(TileSetMap::value_type(tsp.id, tileSet));
    }

    // log
    for (const auto &ts : update) {
        LOG(info2) << "Adding tile set <" << ts.first << ">:\n"
                   << utility::dump(ts.second->getProperties(), "    ");
    }

    // open and add all existing tile sets to the kept sets
    for (const auto &input : properties.inputSets) {
        auto tileSet(openTileSet(rooted(root, input.second.locator)));
        kept.insert(TileSetMap::value_type(input.first, tileSet));
    }

    // open output tile set
    auto output(openTileSet(rooted(root, properties.outputSet.locator)
                            , OpenMode::readWrite));

    // begin transaction
    output->begin();

    if (kept.empty() && (update.size() == 1)) {
        // TODO: no merge at all, just clone in
        output->mergeIn(asList(kept), asList(update));
    } else {
        // merge in tile sets to the output tile set
        output->mergeIn(asList(kept), asList(update));
    }

    // should be fine now

    // copy in tile sets to the storage
    for (const auto &ts : update) {
        LOG(info4) << "Copying tile set <" << ts.first << "> into storage.";

        // add to properties
        // TODO: what tileset type to use?
        Locator dst(DefaultInputType
                    , (fs::path(InputDir) / ts.first).string());
        properties.inputSets[ts.first].locator = dst;

        // clone
        cloneTileSet(rooted(root, dst), ts.second, CreateMode::overwrite);
    }

    // commit changes to output
    output->commit();

    // done
    saveConfig();
}

void Storage::Detail::removeTileSets(const std::vector<std::string> &ids)
{
    TileSetMap kept;
    TileSetMap update;

    for (const auto &id : ids) {
        auto *desc(findInput(id));
        if (!desc) {
            LOGTHROW(err2, NoSuchTileSet)
                << "This storage doesn't contain input tile set <"
                << id << ">.";
        }

        update.insert(TileSetMap::value_type
                      (id, openTileSet(rooted(root, desc->locator))));
    }

    // open and add all tile sets that should be kept to the merge input
    for (const auto &input : properties.inputSets) {
        if (update.find(input.first) == update.end()) {
            auto tileSet(openTileSet(rooted(root, input.second.locator)));
            kept.insert(TileSetMap::value_type(input.first, tileSet));
        }
    }

    // log
    for (const auto &ts : update) {
        LOG(info2) << "Removing tile set <" << ts.first << ">:\n"
                   << utility::dump(ts.second->getProperties(), "    ");
    }

    // open output tile set
    auto output(openTileSet(rooted(root, properties.outputSet.locator)
                            , OpenMode::readWrite));

    // begin transaction
    output->begin();

    // merge out tile sets from the output tile set
    output->mergeOut(asList(kept), asList(update));

    // TODO: remove out tile sets from the storage

    // commit changes to output
    output->commit();
}

// storage

StorageProperties Storage::getProperties() const
{
    return detail().properties;
}

const std::string Storage::getDefaultOutputType()
{
    return DefaultOutputType;
}

std::map<std::string, std::string> Storage::listSupportedDrivers()
{
    return Driver::listSupportedDrivers();
}

Storage::Storage(const fs::path &root, bool readOnly)
    : detail_(new Detail(root, readOnly))
{
    detail().loadConfig();
}

void Storage::addTileSets(const std::vector<Locator> &locators)
{
    return detail().addTileSets(locators);
}

void Storage::removeTileSets(const std::vector<std::string> &ids)
{
    return detail().removeTileSets(ids);
}

} } // namespace vadstena::tilestorage
