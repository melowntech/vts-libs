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
#ifndef vtslibs_vts_tileset_driver_cache_hpp_included_
#define vtslibs_vts_tileset_driver_cache_hpp_included_

#include <set>
#include <map>

#include <boost/noncopyable.hpp>
#include <boost/optional.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/uuid/uuid.hpp>

#include "utility/raise.hpp"

#include "../../../storage/io.hpp"
#include "../../../storage/tilar.hpp"
#include "../../../storage/streams.hpp"
#include "../../../storage/resources.hpp"
#include "../../../storage/error.hpp"
#include "../../basetypes.hpp"

#include "options.hpp"

namespace vtslibs { namespace vts { namespace driver {

using storage::OStream;
using storage::IStream;
using storage::File;
using storage::TileFile;
using storage::FileStat;
using storage::Resources;
using storage::NullWhenNotFound_t;
using storage::NullWhenNotFound;

namespace fs = boost::filesystem;

class Cache : boost::noncopyable {
public:
    Cache(const fs::path &root, const PlainOptions &options
          , bool readOnly);

    ~Cache();

    IStream::pointer input(const TileId tileId, TileFile type);

    IStream::pointer input(const TileId tileId, TileFile type
                           , const NullWhenNotFound_t&);

    OStream::pointer output(const TileId tileId, TileFile type);

    std::size_t size(const TileId tileId, TileFile type);

    FileStat stat(const TileId tileId, TileFile type);

    storage::Resources resources();

    void flush();

    bool readOnly() const { return readOnly_; }

    void makeReadOnly() { readOnly_ = true; }

private:
    struct Archives;

    Archives& getArchives(TileFile type);

    const fs::path &root_;
    const PlainOptions options_;
    bool readOnly_;

    std::unique_ptr<Archives> tiles_;
    std::unique_ptr<Archives> metatiles_;
    std::unique_ptr<Archives> navtiles_;
};

inline Cache::Archives& Cache::getArchives(TileFile type)
{
    switch (type) {
    case TileFile::meta: return *metatiles_;
    case TileFile::mesh: case TileFile::atlas: return *tiles_;
    case TileFile::navtile: return *navtiles_;
    default: break;
    }

    utility::raise<storage::NoSuchTile>("File type <%s> unsupported by "
                                        "this driver", type);
    throw;
}

} } } // namespace vtslibs::vts::driver

#endif // vtslibs_vts_tileset_driver_cache_hpp_included_
