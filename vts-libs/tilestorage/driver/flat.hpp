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
#ifndef vtslibs_tilestorage_driver_flat_hpp_included_
#define vtslibs_tilestorage_driver_flat_hpp_included_

#include "./fsbased.hpp"
#include "./factory.hpp"

namespace vtslibs { namespace tilestorage {

class FlatDriver : public FsBasedDriver {
public:
    FlatDriver(const boost::filesystem::path &root, CreateMode mode
               , const CreateProperties &properties)
        : FsBasedDriver(root, mode, properties)
    {}

    /** Opens storage.
     */
    FlatDriver(const boost::filesystem::path &root, OpenMode mode
               , const DetectionContext &context)
        : FsBasedDriver(root, mode, context)
    {}

    virtual ~FlatDriver() {}

    static const std::string help;

    VADSTENA_TILESTORAGE_DRIVER_FACTORY("flat", FlatDriver);

private:
    virtual boost::filesystem::path fileDir_impl(File, const fs::path&)
        const UTILITY_OVERRIDE
    {
        return {};
    }

    virtual boost::filesystem::path
    fileDir_impl(const TileId&, TileFile, const fs::path&)
        const UTILITY_OVERRIDE
    {
        return {};
    }

    virtual DriverProperties properties_impl() const UTILITY_OVERRIDE {
        return { Factory::staticType(), {} };
    }
};

} } // namespace vtslibs::tilestorage

#endif // vtslibs_tilestorage_driver_flat_hpp_included_
