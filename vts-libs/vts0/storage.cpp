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
#include "utility/streams.hpp"

#include "jsoncpp/io.hpp"

#include "storage.hpp"
#include "tileset.hpp"
#include "../vts0.hpp"
#include "tileset-advanced.hpp"
#include "../storage/error.hpp"
#include "json.hpp"
#include "io.hpp"
#include "driver.hpp"

namespace vtslibs { namespace vts0 {

namespace fs = boost::filesystem;

namespace {
    const std::string InputDir("input");

    const std::string OutputDir("output");

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
            Json::write(f, config);
            f.close();
        } catch (const std::exception &e) {
            LOGTHROW(err2, storage::Error)
                << "Unable to write " << path << " config: "
                << e.what() << ".";
        }
    }

    boost::filesystem::path rooted(const fs::path &root
                                   , const boost::filesystem::path &path) {
        return root / path;
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
                LOGTHROW(err2, storage::StorageAlreadyExists)
                    << "Storage at " << root << " already exists.";
            }
        }

        // create input directory
        create_directories(root / InputDir);

        // create default config
        StorageProperties sp;
        sp.outputSet.path = OutputDir;

        // update create properties
        CreateProperties p(properties.createProperties);
        p.staticSetter().id(OutputDir);

        // create output tile set
        createTileSet(rooted(root, sp.outputSet.path), p, mode);

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

    void addTileSets(const std::vector<boost::filesystem::path> &paths
                     , utility::Runnable *runnable);

    void rebuildOutput();

    void removeTileSets(const std::vector<std::string> &ids
                        , utility::Runnable *runnable);

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

        config = Json::read<storage::FormatError>
            (f, path, "vts0 storage config");
        f.close();
    } catch (const std::exception &e) {
        LOGTHROW(err2, storage::Error)
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

void Storage::Detail::addTileSets
(const std::vector<boost::filesystem::path> &paths
 , utility::Runnable *runnable)
{
    // open output tile set
    auto output(openTileSet(rooted(root, properties.outputSet.path)
                            , OpenMode::readWrite));

    TileSetMap kept;
    TileSetMap update;

    // open and add all new tile sets to the merge input
    for (const auto &path : paths) {
        auto tileSet(openTileSet(path));
        const auto tsp(tileSet->getProperties());

        if (findInput(tsp.id)) {
            LOGTHROW(err2, storage::TileSetAlreadyExists)
                << "This storage already contains input tile set <"
                << tsp.id << ">.";
        }

        if (!output->compatible(*tileSet)) {
            LOGTHROW(err2, storage::IncompatibleTileSet)
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
        auto tileSet(openTileSet(rooted(root, input.second.path)));
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
            LOG(info2) << utility::dump(output->getProperties());

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

            fs::path dst(fs::path(InputDir) / ts.first);

            // add to properties
            properties.inputSets[ts.first].path = dst;

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
        // rethrow exception to fail outside
        throw;
    }

    // done
    saveConfig();

    LOG(info3) << "Add: done";
}

void Storage::Detail::rebuildOutput()
{
    // open output tile set
    auto output(openTileSet(rooted(root, properties.outputSet.path)
                            , OpenMode::readWrite));

    TileSetMap kept; // stays empty, whole output is rebuilt from scratch
    TileSetMap update;

    // open and add all existing tile sets to the update sets
    for (const auto &input : properties.inputSets) {
        auto tileSet(openTileSet(rooted(root, input.second.path)));
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
        // rethrow exception to fail outside
        throw;
    }

    // done
    saveConfig();

    LOG(info3) << "Rebuild: done";
}

// storage

StorageProperties Storage::getProperties() const
{
    return detail().properties;
}

Storage::Storage(const fs::path &root, bool readOnly)
    : detail_(new Detail(root, readOnly))
{
    detail().loadConfig();
}

void Storage::addTileSets(const std::vector<boost::filesystem::path> &paths
                          , utility::Runnable *runnable)
{
    return detail().addTileSets(paths, runnable);
}

void Storage::rebuildOutput()
{
    return detail().rebuildOutput();
}

} } // namespace vtslibs::vts0
