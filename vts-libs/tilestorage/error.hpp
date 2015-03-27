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

struct TileSetNotEmpty : Error {
    TileSetNotEmpty(const std::string &message) : Error(message) {}
};

struct StorageAlreadyExists : Error {
    StorageAlreadyExists(const std::string &message) : Error(message) {}
};

struct FormatError : Error {
    FormatError(const std::string &message) : Error(message) {}
};

struct NoSuchTile : Error {
    NoSuchTile(const std::string &message) : Error(message) {}
};

struct NoSuchFile : Error {
    NoSuchFile(const std::string &message) : Error(message) {}
};

struct ReadOnlyError : Error {
    ReadOnlyError(const std::string &message) : Error(message) {}
};

struct PendingTransaction : Error {
    PendingTransaction(const std::string &message) : Error(message) {}
};

struct IncompatibleTileSet : Error {
    IncompatibleTileSet(const std::string &message) : Error(message) {}
};

struct InvalidSignature : Error {
    InvalidSignature(const std::string &message) : Error(message) {}
};

struct Interrupted : Error {
    Interrupted(const std::string &message) : Error(message) {}
};

struct TileSetNotFlushed : Error {
    TileSetNotFlushed(const std::string &message) : Error(message) {}
};

} } // namespace vadstena::tilestorage

#endif // vadstena_libs_tilestorage_error_hpp_included_
