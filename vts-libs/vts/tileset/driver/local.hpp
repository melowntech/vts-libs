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
#ifndef vtslibs_vts_tileset_driver_local_hpp_included_
#define vtslibs_vts_tileset_driver_local_hpp_included_

#include <set>
#include <map>
#include <memory>

#include "../driver.hpp"

namespace vtslibs { namespace vts { namespace driver {

/** Helper class.
 */
struct LocalDriverBase {
    LocalDriverBase() {}
    LocalDriverBase(const CloneOptions &cloneOptions);
};

class LocalDriver : private LocalDriverBase, public Driver {
private:
    struct PrivateTag {};

public:
    typedef std::shared_ptr<LocalDriver> pointer;

    /** Creates new storage. Existing storage is overwritten only if mode ==
     *  CreateMode::overwrite.
     */
    LocalDriver(const boost::filesystem::path &root
                     , const LocalOptions &options
                     , const CloneOptions &cloneOptions);

    /** Opens storage.
     */
    LocalDriver(const boost::filesystem::path &root
                , const OpenOptions &openOptions
                , const LocalOptions &options);

    virtual ~LocalDriver();

    /** Reencodes local tileset.
     */
    static bool reencode(const boost::filesystem::path &root
                         , const LocalOptions &driverOptions
                         , const ReencodeOptions &options
                         , const std::string &prefix = "");

    /** Async open.
     */
    static void open(const boost::filesystem::path &root
                     , const OpenOptions &openOptions
                     , const LocalOptions &options
                     , const DriverOpenCallback::pointer &callback);

    /** Final async open ctor.
     */
    LocalDriver(PrivateTag, const boost::filesystem::path &root
                , const OpenOptions &openOptions
                , const LocalOptions &options
                , Driver::pointer owned);
private:
    virtual OStream::pointer output_impl(const File type);

    virtual IStream::pointer input_impl(File type) const;

    virtual IStream::pointer input_impl(File type, const NullWhenNotFound_t&)
        const;

    virtual OStream::pointer
    output_impl(const TileId &tileId, TileFile type);

    virtual IStream::pointer
    input_impl(const TileId &tileId, TileFile type) const;

    virtual IStream::pointer
    input_impl(const TileId &tileId, TileFile type, const NullWhenNotFound_t&)
        const;

    virtual void drop_impl();

    virtual void flush_impl();

    virtual FileStat stat_impl(File type) const;

    virtual FileStat stat_impl(const TileId &tileId, TileFile type) const;

    virtual Resources resources_impl() const;

    Driver::pointer clone_impl(const boost::filesystem::path &root
                               , const CloneOptions &cloneOptions) const;

    virtual std::string info_impl() const;

    inline const LocalOptions& options() const {
        return Driver::options<const LocalOptions&>();
    }

    Driver::pointer driver_;
};

} } } // namespace vtslibs::vts::driver

#endif // vtslibs_vts_tileset_driver_local_hpp_included_
