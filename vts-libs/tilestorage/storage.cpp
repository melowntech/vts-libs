#include "utility/streams.hpp"

#include "./storage.hpp"
#include "./tileset.hpp"
#include "../tilestorage.hpp"
#include "./tileset-advanced.hpp"
#include "./error.hpp"
#include "./json.hpp"
#include "./io.hpp"
#include "./driver.hpp"

namespace vadstena { namespace tilestorage {

namespace fs = boost::filesystem;

namespace {
    const std::string InputDir("input");

    const std::string OutputDir("output");

    const std::string DefaultInputType("hash/crc");

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

    void addTileSets(const std::vector<Locator> &locators
                     , utility::Runnable *runnable);

    void rebuildOutput();

    void removeTileSets(const std::vector<std::string> &ids
                        , utility::Runnable *runnable);

    void update();

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

namespace {

void updateTexelSize( TileSet::pointer &output
                    , const TileSetMap &kept
                    , const TileSetMap &update )
{
    float minTexelSize = std::numeric_limits<float>::max();
    for(const auto &mtileset : kept) {
        minTexelSize=std::min( minTexelSize
                             , mtileset.second->getProperties().texelSize);
    }
    for(const auto &mtileset : update) {
        minTexelSize=std::min( minTexelSize
                             , mtileset.second->getProperties().texelSize);
    }

    SettableProperties properties;
    properties.texelSize = minTexelSize;
    SettableProperties::MaskType mask = SettableProperties::Mask::texelSize;

    output->setProperties(properties, mask);
}

} // namespace

void Storage::Detail::addTileSets(const std::vector<Locator> &locators
                                  , utility::Runnable *runnable)
{
    // open output tile set
    auto output(openTileSet(rooted(root, properties.outputSet.locator)
                            , OpenMode::readWrite));

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

        if (!output->compatible(*tileSet)) {
            LOGTHROW(err2, IncompatibleTileSet)
                << "Tile set <" << tsp.id
                << "> is incompatible with this storage.";
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

    try {
        // begin transaction
        output->begin(runnable);

        if (kept.empty() && (update.size() == 1)) {
            LOG(info3)
                << "Copying tile set <" << update.begin()->first
                << "> as an output into the empty storage.";

            // get current properties
            auto thisProps(output->getProperties());
            LOG(info4) << utility::dump(output->getProperties());

            // clone
            cloneTileSet(output, update.begin()->second
                         , CloneOptions(CreateMode::overwrite).staticSetter()

                         // fill static properties to update
                         .id(thisProps.id)
                         .metaLevels(thisProps.metaLevels)
                         .srs(thisProps.srs)
                         .driver(thisProps.driver)

                         .context().settableSetter()

                         // fill settable properties to update
                         .textureQuality(thisProps.textureQuality)

                         // done
                         .context()
                         );
        } else {
            // merge in tile sets to the output tile set
            output->mergeIn(asList(kept), asList(update));
        }

        // should be fine now

        // copy in tile sets to the storage
        for (const auto &ts : update) {
            LOG(info3)
                << "Copying tile set <" << ts.first
                << "> as an input into the storage.";

            // add to properties
            // TODO: what tileset type to use?
            Locator dst(DefaultInputType
                        , (fs::path(InputDir) / ts.first).string());
            properties.inputSets[ts.first].locator = dst;

            // clone
            cloneTileSet(rooted(root, dst), ts.second, CreateMode::overwrite);
        }

        updateTexelSize(output, kept, update);

        // commit changes to output
        output->commit();
    } catch (const std::exception &e) {
        LOG(warn3)
            << "Operation being rolled back due to an error: <"
            << e.what() << ">.";
        output->rollback();
    }

    // done
    saveConfig();

    LOG(info3) << "Add: done";
}

void Storage::Detail::rebuildOutput()
{
    // open output tile set
    auto output(openTileSet(rooted(root, properties.outputSet.locator)
                            , OpenMode::readWrite));

    TileSetMap kept; // stays empty, whole output is rebuilt from scratch
    TileSetMap update;

    // open and add all existing tile sets to the update sets
    for (const auto &input : properties.inputSets) {
        auto tileSet(openTileSet(rooted(root, input.second.locator)));
        update.insert(TileSetMap::value_type(input.first, tileSet));
    }

    try {
        // begin transaction
        output->begin();

        if (kept.empty() && (update.size() == 1)) {
            LOG(info3)
                << "(rebuild) Copying tile set <" << update.begin()->first
                << "> to empty storage.";
            // save properties
            auto props(output->getProperties());
            // clone
            cloneTileSet(output, update.begin()->second
                         , CreateMode::overwrite);

            // renew properties (only texture quality so far)
            output->setProperties
                (props, SettableProperties::Mask::textureQuality);

            // renew metalevels, id, srs
            auto a(output->advancedApi());
            a.rename(props.id);
            a.changeMetaLevels(props.metaLevels);
            a.changeSrs(props.srs);
        } else {
            // merge in tile sets to the output tile set
            output->mergeIn(asList(kept), asList(update));
        }

        updateTexelSize(output, kept, update);

        // commit changes to output
        output->commit();
    } catch (const std::exception &e) {
        LOG(warn3)
            << "Operation being rolled back due to an error: <"
            << e.what() << ">.";
        output->rollback();
    }

    // done
    saveConfig();

    LOG(info3) << "Rebuild: done";
}

void Storage::Detail::removeTileSets(const std::vector<std::string> &ids
                                     , utility::Runnable *runnable)
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

    try {
        // begin transaction
        output->begin(runnable);

        // merge out tile sets from the output tile set
        output->mergeOut(asList(kept), asList(update));

        // remove out tile sets from the storage
        for (const auto &ts : update) {
            LOG(info3)
                << "Removing tile set <" << ts.first << "> from storage.";

            // remove from properties
            properties.inputSets.erase(ts.first);
        }

        updateTexelSize(output, kept, TileSetMap());

        // commit changes to output
        output->commit();
    } catch (const std::exception &e) {
        LOG(warn3)
            << "Operation being rolled back due to an error: <"
            << e.what() << ">.";
        output->rollback();
    }

    // done
    saveConfig();

    // remove out tile sets from the storage
    for (const auto &ts : update) {
        LOG(info3)
            << "Removing tile set <" << ts.first << "> from storage.";

        // remove tileset
        ts.second->drop();
    }

    LOG(info3) << "Remove: done";
}

void Storage::Detail::update()
{
    for (const auto &input : properties.inputSets) {
        LOG(info3) << "Updating input tile set <" << input.first << ">.";
        openTileSet(rooted(root, input.second.locator), OpenMode::readWrite)
            ->update();
    }

    LOG(info3) << "Updating output tile set.";
    openTileSet(rooted(root, properties.outputSet.locator)
                , OpenMode::readWrite)->update();

    LOG(info3) << "Update: done";
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

void Storage::addTileSets(const std::vector<Locator> &locators
                          , utility::Runnable *runnable)
{
    return detail().addTileSets(locators, runnable);
}

void Storage::rebuildOutput()
{
    return detail().rebuildOutput();
}

void Storage::removeTileSets(const std::vector<std::string> &ids
                             , utility::Runnable *runnable)
{
    return detail().removeTileSets(ids, runnable);
}

void Storage::update()
{
    return detail().update();
}

} } // namespace vadstena::tilestorage
