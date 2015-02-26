#include <fstream>

#include <boost/filesystem.hpp>
#include <boost/format.hpp>
#include <boost/utility/in_place_factory.hpp>

#include "utility/streams.hpp"
#include "utility/path.hpp"

#include "./tilardriver.hpp"
#include "../io.hpp"
#include "../error.hpp"
#include "../config.hpp"

#include "tilestorage/browser/index.html.hpp"
#include "tilestorage/browser/index-offline.html.hpp"
#include "tilestorage/browser/skydome.jpg.hpp"

namespace vadstena { namespace tilestorage {

namespace fs = boost::filesystem;

namespace {
    const std::string ConfigName("mapConfig.json");
    const std::string TileIndexName("index.bin");
    const std::string TransactionRoot("tx");

    const std::string filePath(File type)
    {
        switch (type) {
        case File::config: return ConfigName;
        case File::tileIndex: return TileIndexName;
        }
        throw "unknown file type";
    }

} // namespace

TilarDriver::TilarDriver(const boost::filesystem::path &root
                         , CreateMode mode, const StaticProperties&)
    : Driver(false)
    , root_(root), tmp_(root / TransactionRoot)
{
    if (!create_directories(root_)) {
        // directory already exists -> fail if mode says so
        if (mode == CreateMode::failIfExists) {
            LOGTHROW(err2, TileSetAlreadyExists)
                << "Tile set at " << root_ << " already exists.";
        }
    }

    // write extra files
    writeExtraFiles();
}

TilarDriver::TilarDriver(const boost::filesystem::path &root
                         , OpenMode mode)
    : Driver(mode == OpenMode::readOnly)
    , root_(root), tmp_(root / TransactionRoot)
{
}

TilarDriver::~TilarDriver()
{
#if 0
    if (tx_) {
        LOG(warn3) << "Active transaction on driver close; rolling back.";
        try {
            rollback_impl();
        } catch (const std::exception &e) {
            LOG(warn3)
                << "Error while trying to destroy active transaction on "
                "driver close: <" << e.what() << ">.";
        }
    }
#endif
}

void TilarDriver::writeExtraFiles()
{
    // write convenience browser
    utility::write(root_ / "index.html", browser::index_html);
    utility::write(root_ / "index-offline.html", browser::index_offline_html);
    utility::write(root_ / "skydome.jpg", browser::skydome_jpg);
}

OStream::pointer TilarDriver::output_impl(File type)
{
    (void) type;
    return {};
}

IStream::pointer TilarDriver::input_impl(File type) const
{
    (void) type;
    return {};
}

OStream::pointer TilarDriver::output_impl(const TileId tileId, TileFile type)
{
    (void) tileId; (void) type;
    return {};
}

IStream::pointer TilarDriver::input_impl(const TileId tileId, TileFile type)
    const
{
    (void) tileId; (void) type;
    return {};
}

void TilarDriver::remove_impl(const TileId tileId, TileFile type)
{
    (void) tileId; (void) type;
}

void TilarDriver::begin_impl()
{
}

void TilarDriver::commit_impl()
{
}

void TilarDriver::rollback_impl()
{
}

void TilarDriver::drop_impl()
{
    // remove whole tmp directory
    remove_all(root_);
}

void TilarDriver::update_impl()
{
    // write extra files (i.e. browser)
    writeExtraFiles();
}

DriverProperties TilarDriver::properties_impl() const
{
    // TODO: return options
    return { Factory::staticType(), {} };
}

std::string TilarDriver::detectType_impl(const std::string &location)
{
    try {
        // try load config
        std::ifstream f;
        f.exceptions(std::ios::badbit | std::ios::failbit);
        f.open((fs::path(location) / filePath(File::config)).string());
        const auto p(tilestorage::loadConfig(f));
        f.close();
        return p.driver.type;
    } catch (const std::exception&) {}
    return {};
}

const std::string TilarDriver::help
("Filesystem-based storage driver with deep directory structure. "
 "Tiles and metatiles are stored inside Tile Archives.");

} } // namespace vadstena::tilestorage
