#include "../tileset.hpp"
#include "./detail.hpp"
#include "./driver.hpp"
#include "./config.hpp"

namespace fs = boost::filesystem;

namespace vadstena { namespace vts {

StaticProperties TileSet::getProperties() const
{
    return {};
}

TileSet::TileSet(const std::shared_ptr<Driver> &driver)
{
    (void) driver;
}

TileSet::TileSet(const std::shared_ptr<Driver> &driver
                 , const StaticProperties &properties)
{
    (void) driver;
    (void) properties;
}

TileSet::~TileSet()
{
}

Mesh TileSet::getMesh(const TileId &tileId) const
{
    (void) tileId;
    return {};
}

void TileSet::setMesh(const TileId &tileId, const Mesh &mesh)
{
    (void) tileId;
    (void) mesh;
}

void TileSet::getAtlas(const TileId &tileId, Atlas &atlas)
{
    (void) tileId; (void) atlas;
}

void TileSet::setAtlas(const TileId &tileId, const Atlas &atlas) const
{
    (void) tileId; (void) atlas;
}

MetaNode TileSet::getMetaNode(const TileId &tileId) const
{
    (void) tileId;
    return {};
}

void TileSet::setMetaNode(const TileId &tileId, const MetaNode node)
{
    (void) tileId;
    (void) node;
}

bool TileSet::exists(const TileId &tileId) const
{
    (void) tileId;
    return false;
}

void TileSet::flush()
{
}

void TileSet::begin(utility::Runnable *runnable)
{
    (void) runnable;
}

void TileSet::commit()
{
}

void TileSet::rollback()
{
}

void TileSet::watch(utility::Runnable *runnable)
{
    (void) runnable;
}

bool TileSet::inTx() const
{
    return false;
}

bool TileSet::empty() const
{
    return true;
}

void TileSet::drop()
{
}

LodRange TileSet::lodRange() const
{
    return {};
}

struct TileSet::Factory
{
    static TileSet create(const fs::path &path
                          , const StaticProperties &properties
                          , CreateMode mode)
    {
        // we are using binaryOrder = 5 :)
        auto driver(std::make_shared<Driver>(path, mode, 5));
        return TileSet(driver, properties);
    }

    static TileSet open(const fs::path &path)
    {
        auto driver(std::make_shared<Driver>(path));
        return TileSet(driver);
    }
};

TileSet createTileSet(const boost::filesystem::path &path
                      , const StaticProperties &properties
                      , CreateMode mode)
{
    return TileSet::Factory::create(path, properties, mode);
}

TileSet openTileSet(const boost::filesystem::path &path)
{
    return TileSet::Factory::open(path);
}

TileSet::Detail::Detail(const Driver::pointer &driver)
    : readOnly(true), driver(driver), propertiesChanged(false)
    , metadataChanged(false)
{
    loadConfig();

    // TODO: load tile index
}

TileSet::Detail::Detail(const Driver::pointer &driver
                        , const StaticProperties &properties)
    : readOnly(false), driver(driver), propertiesChanged(false)
    , metadataChanged(false)
{
    if (properties.id.empty()) {
        LOGTHROW(err2, storage::FormatError)
            << "Cannot create tile set without valid id.";
    }

    if (properties.referenceFrame.empty()) {
        LOGTHROW(err2, storage::FormatError)
            << "Cannot create tile set without valid reference frame.";
    }

    // build initial properties
    static_cast<StaticProperties&>(this->properties) = properties;
    this->properties.driverOptions = driver->options();
    savedProperties = this->properties;

    // tile index must be properly initialized
    // tileIndex = {};

    saveConfig();
}

void TileSet::Detail::loadConfig()
{
    try {
        // load config
        auto f(driver->input(File::config));
        const auto p(vts::loadConfig(*f));
        f->close();

        // set
        savedProperties = properties = p;
    } catch (const std::exception &e) {
        LOGTHROW(err2, storage::Error)
            << "Unable to read config: <" << e.what() << ">.";
    }
}

void TileSet::Detail::saveConfig()
{
    // save json
    try {
        driver->wannaWrite("save config");
        auto f(driver->output(File::config));
        vts::saveConfig(*f, properties);
        f->close();
    } catch (const std::exception &e) {
        LOGTHROW(err2, storage::Error)
            << "Unable to write config: <" << e.what() << ">.";
    }

    // done; remember saved properties and go on
    savedProperties = properties;
    propertiesChanged = false;
}

} } // namespace vadstena::vts
