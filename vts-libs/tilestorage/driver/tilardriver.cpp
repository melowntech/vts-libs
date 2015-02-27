#include <stdexcept>
#include <limits>
#include <type_traits>
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
    const std::string KeyBinarySize("binarySize");
    const std::uint8_t DefaultBinarySize(5);

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

    const char *extension(TileFile type)
    {
        switch (type) {
        case TileFile::meta:
            return "metatiles";
        case TileFile::mesh:
        case TileFile::atlas:
            return "tiles";
        }
        throw "unknown tile file type";
    }

    fs::path filePath(const Index &index, TileFile type)
    {
        return str(boost::format("%s-%07d-%07d.%s")
                   % index.lod % index.easting % index.northing
                   % extension(type));
    }

    template <typename T, class Enable = void>
    T getOption(const DriverProperties::Options&, const std::string &
                , const boost::optional<T>& = boost::none);

    template
    <typename T
     , class = typename std::enable_if<std::is_unsigned<T>::value>::type>
    T getOption(const DriverProperties::Options &options
                , const std::string &key
                , const boost::optional<T> &defaultValue = boost::none)
    {
        auto foptions(options.find(key));
        if (foptions == options.end()) {
            if (!defaultValue) {
                LOGTHROW(err2, std::runtime_error)
                    << "Option <" << key << "> not found "
                    "and no default value has been provided.";
            }
            return *defaultValue;
        }

        std::uint64_t value{};
        try {
            value = boost::any_cast<std::uint64_t>(foptions->second);
        } catch (const boost::bad_any_cast&) {
            LOGTHROW(err2, std::logic_error)
                << "Options value <" << key << "> is not an unsigned integer.";
        }

        if (value > std::numeric_limits<T>::max()) {
            LOGTHROW(err2, std::logic_error)
                << "Options value <" << key << "> is doesn't fit into "
                "requested type.";
        }
        return static_cast<T>(value);
    }
} // namespace

TilarDriver::Options::Options(const StaticProperties &properties)
    : baseTileSize(properties.baseTileSize)
    , alignment(properties.alignment)
    , binarySize(getOption<decltype(binarySize)>
                 (properties.driver.options, KeyBinarySize))
{}

TilarDriver::Options::Options(const StaticProperties &properties
                              , bool)
    : baseTileSize(properties.baseTileSize)
    , alignment(properties.alignment)
    , binarySize(getOption<decltype(binarySize)>
                 (properties.driver.options, KeyBinarySize
                  , DefaultBinarySize))
{}

TilarDriver::TilarDriver(const boost::filesystem::path &root
                         , CreateMode mode, const StaticProperties &properties)
    : Driver(false)
    , root_(root), tmp_(root / TransactionRoot)
    , options_(properties, true)
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
    , options_(tilestorage::loadConfig(root_ / filePath(File::config)))
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
    // TODO: add tx support
    const auto name(filePath(type));
    const auto path(root_ / name);
    LOG(info1) << "Saving to " << path << ".";
    return std::make_shared<FileOStream>(path);
}

IStream::pointer TilarDriver::input_impl(File type) const
{
    // TODO: add tx support
    const auto name(filePath(type));
    const auto path(root_ / name);
    LOG(info1) << "Loading from " << path << ".";
    return std::make_shared<FileIStream>(path);
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
    DriverProperties dp;
    dp.type = Factory::staticType();
    dp.options[KeyBinarySize] = boost::any(std::uint64_t(options_.binarySize));
    return dp;
}

std::string TilarDriver::detectType_impl(const std::string &location)
{
    try {
        // try load config
        return tilestorage::loadConfig
            (fs::path(location) / filePath(File::config)).driver.type;
    } catch (const std::exception&) {}
    return {};
}

const std::string TilarDriver::help
("Filesystem-based storage driver with deep directory structure. "
 "Tiles and metatiles are stored inside Tile Archives.");

} } // namespace vadstena::tilestorage
