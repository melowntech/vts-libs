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
/**
 * \file storage/error.hpp
 * \author Vaclav Blazek <vaclav.blazek@citationtech.net>
 *
 * Tile storage error types.
 */

#ifndef vtslibs_storage_error_hpp_included_
#define vtslibs_storage_error_hpp_included_

#include <stdexcept>
#include <string>

namespace vtslibs { namespace storage {

struct Error : std::runtime_error {
    Error(const std::string &message) : std::runtime_error(message) {}
};

struct BadFileFormat : std::runtime_error {
    BadFileFormat(const std::string &message) : std::runtime_error(message) {}
};

struct VersionError : std::runtime_error {
    VersionError(const std::string &message) : std::runtime_error(message) {}
};

struct InvalidId : Error {
    InvalidId(const std::string &message) : Error(message) {}
};

struct NoSuchTileSet : Error {
    NoSuchTileSet(const std::string &message) : Error(message) {}
};

struct NoSuchStorage : Error {
    NoSuchStorage(const std::string &message) : Error(message) {}
};

struct NoSuchStorageView : Error {
    NoSuchStorageView(const std::string &message) : Error(message) {}
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

struct InconsistentInput : Error {
    InconsistentInput(const std::string &message) : Error(message) {}
};

struct Interrupted : Error {
    Interrupted(const std::string &message) : Error(message) {}
};

struct TileSetNotFlushed : Error {
    TileSetNotFlushed(const std::string &message) : Error(message) {}
};

struct KeyError : Error {
    KeyError(const std::string &message) : Error(message) {}
};

struct IOError : Error {
    IOError(const std::string &message) : Error(message) {}
};

struct Corrupted : Error {
    Corrupted(const std::string &message) : Error(message) {}
};

struct Unimplemented : Error {
    Unimplemented(const std::string &message) : Error(message) {}
};

} } // namespace vtslibs::storage

#endif // vtslibs_storage_error_hpp_included_
