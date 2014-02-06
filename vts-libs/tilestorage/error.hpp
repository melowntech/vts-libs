/**
 * \file tilestorage/error.hpp
 * \author Vaclav Blazek <vaclav.blazek@citationtech.net>
 *
 * Tile storage error types.
 */

#ifndef vadstena_libs_tilestorage_error_hpp_included_
#define vadstena_libs_tilestorage_error_hpp_included_

#include <stdexcept>
#include <string>

namespace vadstena { namespace tilestorage {

struct Error : std::runtime_error {
    Error(const std::string &message) : std::runtime_error(message) {}
};

struct NoSuchTileSet : Error {
    NoSuchTileSet(const std::string &message) : Error(message) {}
};

struct TileSetAlreadyExists : Error {
    TileSetAlreadyExists(const std::string &message) : Error(message) {}
};

struct FormatError : Error {
    FormatError(const std::string &message) : Error(message) {}
};

struct NoSuchTile : Error {
    NoSuchTile(const std::string &message) : Error(message) {}
};

} } // namespace vadstena::tilestorage

#endif // vadstena_libs_tilestorage_error_hpp_included_
