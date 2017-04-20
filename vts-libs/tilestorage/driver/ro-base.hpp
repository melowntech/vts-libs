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
#ifndef vtslibs_tilestorage_driver_ro_base_hpp_included_
#define vtslibs_tilestorage_driver_ro_base_hpp_included_

#include "../driver.hpp"
#include "../../storage/error.hpp"

#include "dbglog/dbglog.hpp"

#include "utility/gccversion.hpp"

namespace vtslibs { namespace tilestorage {

namespace fs = boost::filesystem;

class ReadOnlyDriver : public Driver {
public:
    ReadOnlyDriver(CreateMode, const CreateProperties&)
        : Driver(true)
    {
        LOGTHROW(err2, storage::ReadOnlyError)
            << "This driver supports read access only.";
    }

    /** Opens storage.
     */
    ReadOnlyDriver(bool readOnly)
        : Driver(readOnly)
    {
        if (!readOnly) {
            LOGTHROW(err2, storage::ReadOnlyError)
                << "This driver supports read access only.";
        }
    }

private:
    virtual OStream::pointer output_impl(File) UTILITY_OVERRIDE {
        LOGTHROW(err2, storage::ReadOnlyError)
            << "This driver supports read access only.";
        return {};
    }

    virtual OStream::pointer
    output_impl(const TileId, TileFile) UTILITY_OVERRIDE {
        LOGTHROW(err2, storage::ReadOnlyError)
            << "This driver supports read access only.";
        return {};
    }

    virtual void remove_impl(const TileId, TileFile)
        UTILITY_OVERRIDE
    {
        LOGTHROW(err2, storage::ReadOnlyError)
            << "This driver supports read access only.";
    }

    virtual void begin_impl() UTILITY_OVERRIDE {
        LOGTHROW(err2, storage::ReadOnlyError)
            << "This driver supports read access only.";
    }

    virtual void commit_impl() UTILITY_OVERRIDE {
        LOGTHROW(err2, storage::ReadOnlyError)
            << "This driver supports read access only.";
    }

    virtual void rollback_impl() UTILITY_OVERRIDE {
        LOGTHROW(err2, storage::ReadOnlyError)
            << "This driver supports read access only.";
    }

    virtual void drop_impl() UTILITY_OVERRIDE {
        LOGTHROW(err2, storage::ReadOnlyError)
            << "This driver supports read access only.";
    }
};

} } // namespace vtslibs::tilestorage

#endif // vtslibs_tilestorage_driver_ro_base_hpp_included_
