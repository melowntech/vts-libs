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
#ifndef vtslibs_tilestorage_driver_tar_hpp_included_
#define vtslibs_tilestorage_driver_tar_hpp_included_

#include <set>
#include <map>

#include <boost/optional.hpp>
#include <boost/filesystem/path.hpp>

#include "utility/tar.hpp"

#include "./ro-base.hpp"
#include "./factory.hpp"

namespace vtslibs { namespace tilestorage {

namespace fs = boost::filesystem;

class TarDriver : public ReadOnlyDriver {
public:
    TarDriver(const fs::path&, CreateMode mode
              , const CreateProperties &properties)
        : ReadOnlyDriver(mode, properties) {}

    /** Opens storage.
     */
    TarDriver(const fs::path &root, OpenMode mode
              , const DetectionContext &context);

    virtual ~TarDriver();

    static const std::string help;

    VADSTENA_TILESTORAGE_DRIVER_FACTORY("tar", TarDriver);

    struct Record {
        std::string path;
        std::size_t block;
        std::size_t size;
        std::time_t time;

        Record(std::string path = "", std::size_t block = 0
               , std::size_t size = 0, std::time_t time = 0)
            : path(path), block(block), size(size), time(time)
        {}
    };

    static std::string detectType_impl(DetectionContext &context
                                       , const std::string &location);

private:
    virtual IStream::pointer input_impl(File type) const UTILITY_OVERRIDE;

    virtual IStream::pointer
    input_impl(const TileId tileId, TileFile type) const UTILITY_OVERRIDE;

    virtual FileStat stat_impl(File type) const UTILITY_OVERRIDE;

    virtual FileStat stat_impl(const TileId tileId, TileFile type)
        const UTILITY_OVERRIDE;

    virtual bool externallyChanged_impl() const UTILITY_OVERRIDE;

    virtual DriverProperties properties_impl() const UTILITY_OVERRIDE {
        return { Factory::staticType(), {} };
    }

    virtual Resources resources_impl() const UTILITY_OVERRIDE {
        return { 1, 0 };
    }

    const fs::path tarPath_;
    mutable utility::tar::Reader reader_;

    typedef std::map<TileId, Record> FileMap;
    FileMap meshMap_;
    FileMap atlasMap_;
    FileMap metaMap_;
    Record indexFile_;
    Record configFile_;

    /** Information about tar file when tileset was open in read-only mode.
     */
    FileStat openStat_;
};

} } // namespace vtslibs::tilestorage

#endif // vtslibs_tilestorage_driver_tar_hpp_included_
